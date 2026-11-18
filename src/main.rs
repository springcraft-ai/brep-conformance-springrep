//! The springrep adapter: answers the B-rep protocol's requests on stdin and stdout, so
//! the conformance suite, or any other client, can drive this kernel.

use std::collections::HashMap;
use std::f64::consts::TAU;
use std::process::ExitCode;

use brep_protocol::{Kernel, Refusal, model, model::Primitive, profile};
use springrep::{
    blend, boolean, check,
    distance::{self, Selection},
    edit,
    geom::{BSpline, Curve},
    loft::{Section, loft},
    math::{Frame, Transform, Vec2, Vec3},
    measure, overlap, primitive,
    profile::{NamedSegment, Profile, ProfileLoop, Segment},
    sweep::{self, Leg, extrude, revolve},
    tol,
    topo::{Brep, EdgeId, FaceId},
};

struct Springrep;

/// The kernel's error as the protocol's refusal, its kind the kernel's own.
fn refusal(e: springrep::Error) -> Refusal {
    use springrep::Error;
    let reason = e.to_string();
    match e {
        Error::InvalidInput(_) => Refusal::invalid(reason),
        Error::Unsupported(_) => Refusal::unsupported(reason),
        Error::Numerical(_) | Error::InvalidResult(_) => Refusal::failed(reason),
    }
}

fn v3(a: model::Vec3) -> Vec3 {
    Vec3::new(a[0], a[1], a[2])
}

fn seg(segment: Segment, name: &str) -> NamedSegment {
    NamedSegment::new(segment, name)
}

/// The suite's profile as this kernel's, each loop's segments named by their place in it.
fn sketch(p: &profile::Profile) -> Result<Profile, Refusal> {
    let segment = |s: &profile::Seg| -> Result<Segment, Refusal> {
        if let profile::Seg::Spline(sp) = s {
            if sp.degree != 3 {
                return Err(Refusal::unsupported(format!(
                    "a periodic spline of degree {}; only cubic",
                    sp.degree
                )));
            }
            let ctrl = sp.ctrl.iter().map(|c| Vec3::new(c[0], c[1], 0.0)).collect();
            let curve = BSpline::periodic_cubic(sp.knots.clone(), ctrl).map_err(refusal)?;
            return Ok(Segment::Spline {
                curve,
                reversed: false,
            });
        }
        Ok(match *s {
            profile::Seg::Line { a, b } => Segment::Line {
                a: Vec2::new(a[0], a[1]),
                b: Vec2::new(b[0], b[1]),
            },
            profile::Seg::Arc {
                center,
                radius,
                sweep,
                ..
            } if sweep.abs() >= TAU => Segment::Circle {
                center: Vec2::new(center[0], center[1]),
                radius,
                ccw: sweep > 0.0,
            },
            profile::Seg::Arc {
                center,
                radius,
                start,
                sweep,
            } => Segment::Arc {
                center: Vec2::new(center[0], center[1]),
                radius,
                start,
                sweep,
            },
            profile::Seg::Conic { a, apex, b, w } => Segment::conic(
                Vec2::new(a[0], a[1]),
                Vec2::new(apex[0], apex[1]),
                Vec2::new(b[0], b[1]),
                w,
            )
            .map_err(refusal)?,
            profile::Seg::Gn { .. } => return Err(Refusal::unsupported("a blend-section segment")),
            profile::Seg::Spline(_) => unreachable!(),
        })
    };
    // Numbered through the whole profile: a hole's faces must not share names with the outline's,
    // or a pair of names would pick edges on both.
    let mut next = 0..;
    let loops = p
        .loops
        .iter()
        .map(|l| {
            let segments = l
                .iter()
                .map(|s| Ok(seg(segment(s)?, &format!("s{}", next.next().unwrap_or(0)))))
                .collect::<Result<_, Refusal>>()?;
            Ok(ProfileLoop { segments })
        })
        .collect::<Result<_, Refusal>>()?;
    Ok(Profile { loops })
}

/// The edge each point lies on.
fn edges_at(b: &Brep, points: &[model::Vec3]) -> Result<Vec<EdgeId>, Refusal> {
    points
        .iter()
        .map(|&p| match measure::edge_at(b, v3(p)) {
            Some((e, d)) if d <= 10.0 * tol::LINEAR => Ok(e),
            _ => Err(Refusal::invalid(format!("no edge passes through {p:?}"))),
        })
        .collect()
}

/// The face each point lies on.
fn faces_at(b: &Brep, points: &[model::Vec3]) -> Result<Vec<FaceId>, Refusal> {
    points.iter().map(|&p| face_at(b, p)).collect()
}

/// The face a point lies on, within the contract's `10·ε` of it.
fn face_at(b: &Brep, p: model::Vec3) -> Result<FaceId, Refusal> {
    match measure::face_at(b, v3(p)) {
        Some((f, d)) if d <= 10.0 * tol::LINEAR => Ok(f),
        _ => Err(Refusal::invalid(format!("no face passes through {p:?}"))),
    }
}

/// The sketch plane holding the frame's z axis, with x along the frame's x: revolving a profile
/// drawn in it about that axis turns it round the frame's z.
fn meridian(f: &model::Frame) -> Result<Frame, Refusal> {
    Frame::new(v3(f.origin), -v3(f.y), v3(f.x)).map_err(refusal)
}

/// The suite's frame as the kernel's.
fn own(f: &model::Frame) -> Result<Frame, Refusal> {
    Frame::new(v3(f.origin), v3(f.z), v3(f.x)).map_err(refusal)
}

impl Kernel for Springrep {
    type Solid = Brep;

    fn name(&self) -> String {
        "springrep".into()
    }

    fn version(&self) -> String {
        springrep::VERSION.into()
    }

    fn tolerance(&self) -> f64 {
        tol::LINEAR
    }

    fn make(&self, p: &Primitive) -> Result<Brep, Refusal> {
        let f = p.frame();
        let e = refusal;
        match *p {
            Primitive::Box { size, .. } => primitive::block(&own(&f)?, v3(size), "box").map_err(e),
            Primitive::Cylinder { radius, height, .. } => {
                primitive::cylinder(&own(&f)?, radius, height, "cyl").map_err(e)
            }
            Primitive::Sphere { radius, .. } => {
                primitive::sphere(&own(&f)?, radius, "sph").map_err(e)
            }
            Primitive::Torus { major, minor, .. } => {
                primitive::torus(&own(&f)?, major, minor, "tor").map_err(e)
            }
            Primitive::Cone {
                bottom,
                top,
                height,
                ..
            } => primitive::cone(&own(&f)?, bottom, top, height, "cone").map_err(e),
            Primitive::Extrude {
                ref profile,
                height,
                ..
            } => {
                let frame = Frame::new(v3(f.origin), v3(f.z), v3(f.x)).map_err(e)?;
                extrude(&sketch(profile)?, &frame, height, "ext").map_err(e)
            }
            Primitive::Revolve {
                ref profile, angle, ..
            } => revolve(
                &sketch(profile)?,
                &meridian(&f)?,
                v3(f.origin),
                v3(f.z),
                angle,
                "rev",
            )
            .map_err(e),
            Primitive::Screw {
                ref profile,
                rise,
                angle,
                ..
            } => {
                let motion = sweep::Helix {
                    origin: v3(f.origin),
                    axis: v3(f.z),
                    rise,
                    sign: 1.0,
                };
                sweep::helix(&sketch(profile)?, &meridian(&f)?, &motion, angle, "hel").map_err(e)
            }
            Primitive::Sweep {
                ref profile,
                ref path,
                ..
            } => {
                // The suite's bends turn toward the sketch's x, about its y.
                let frame = own(&f)?;
                let axis = frame.y;
                let legs: Vec<Leg> = path
                    .iter()
                    .map(|seg| match *seg {
                        model::PathSeg::Line { length } => Leg::Line { length },
                        model::PathSeg::Arc { radius, angle } => Leg::Arc {
                            radius,
                            angle,
                            axis,
                        },
                    })
                    .collect();
                sweep::along(&sketch(profile)?, &frame, &legs, "swp").map_err(e)
            }
            Primitive::SweepPath {
                ref profile,
                ref path,
                twist,
                ..
            } => {
                let own = own(&f)?;
                let ctrl = path
                    .ctrl
                    .iter()
                    .map(|c| own.to_world(Vec3::from_array(*c)))
                    .collect();
                let curve =
                    Curve::BSpline(BSpline::new(path.degree, path.knots.clone(), ctrl).map_err(e)?);
                sweep::along_curve(&sketch(profile)?, &own, &curve, path.domain(), twist, "swp")
                    .map_err(e)
            }
            Primitive::Slab {
                size,
                ref knots_u,
                ref knots_v,
                ref heights,
                ..
            } => {
                let fr = Frame::new(v3(f.origin), v3(f.z), v3(f.x)).map_err(e)?;
                springrep::primitive::slab(&fr, size, (knots_u, knots_v), heights, "slab")
                    .map_err(e)
            }
            Primitive::Loft {
                ref profile,
                scale,
                height,
                ..
            } => {
                // The two sections go to the kernel's loft: the top one is the protocol crate's
                // scaled copy, the same for every adapter.
                let base = Frame::new(v3(f.origin), v3(f.z), v3(f.x)).map_err(e)?;
                let top =
                    Frame::new(v3(f.origin) + v3(f.z) * height, v3(f.z), v3(f.x)).map_err(e)?;
                let sections = [
                    Section {
                        profile: sketch(profile)?,
                        frame: base,
                    },
                    Section {
                        profile: sketch(&profile.scaled(scale))?,
                        frame: top,
                    },
                ];
                loft(&sections, true, "loft").map_err(e)
            }
        }
    }

    fn union(&self, a: &Brep, b: &Brep) -> Result<Brep, Refusal> {
        boolean::union(a, b).map_err(refusal)
    }

    fn intersection(&self, a: &Brep, b: &Brep) -> Result<Brep, Refusal> {
        boolean::intersection(a, b).map_err(refusal)
    }

    fn difference(&self, a: &Brep, b: &Brep) -> Result<Brep, Refusal> {
        boolean::difference(a, b).map_err(refusal)
    }

    fn union_all(&self, solids: &[&Brep]) -> Result<Brep, Refusal> {
        boolean::union_all(solids).map_err(refusal)
    }

    fn difference_all(&self, target: &Brep, tools: &[&Brep]) -> Result<Brep, Refusal> {
        boolean::difference_all(target, tools).map_err(refusal)
    }

    fn classify(&self, s: &Brep, points: &[model::Vec3]) -> Result<Vec<model::Side>, Refusal> {
        use springrep::rays::{Rays, Side};
        let rays = Rays::new(s).map_err(refusal)?;
        let side = |p: &model::Vec3| match rays.side(v3(*p)) {
            Side::Inside => model::Side::In,
            Side::On => model::Side::On,
            Side::Outside => model::Side::Out,
        };
        Ok(points.iter().map(side).collect())
    }

    fn closest(&self, s: &Brep, points: &[model::Vec3]) -> Result<Vec<model::Vec3>, Refusal> {
        let nearest = distance::Nearest::solid(s).map_err(refusal)?;
        points
            .iter()
            .map(|&p| {
                let found = nearest.to(v3(p)).map_err(refusal)?;
                Ok(found.on_b.to_array())
            })
            .collect()
    }

    fn volume(&self, s: &Brep) -> Result<f64, Refusal> {
        Ok(measure::volume(s))
    }

    fn area(&self, s: &Brep) -> Result<f64, Refusal> {
        Ok(measure::area(s))
    }

    fn check(&self, s: &Brep) -> Result<Vec<String>, Refusal> {
        Ok(check::check(s).into_iter().map(|i| i.message).collect())
    }

    fn fillet(&self, s: &Brep, edges: &[model::Vec3], radius: f64) -> Result<Brep, Refusal> {
        let ids = edges_at(s, edges)?;
        blend::fillet_edges(s, &ids, radius, "fil").map_err(refusal)
    }

    fn fillet_radii(
        &self,
        s: &Brep,
        edges: &[model::Vec3],
        radii: &[f64],
    ) -> Result<Brep, Refusal> {
        if edges.len() != radii.len() {
            return Err(Refusal::failed(format!(
                "{} edges but {} radii",
                edges.len(),
                radii.len()
            )));
        }
        let ids = edges_at(s, edges)?;
        let pairs: Vec<(EdgeId, f64)> = ids.into_iter().zip(radii.iter().copied()).collect();
        blend::fillet_radii(s, &pairs, "blend").map_err(refusal)
    }

    fn fillet_setback(
        &self,
        s: &Brep,
        edges: &[model::Vec3],
        radius: f64,
        setback: f64,
    ) -> Result<Brep, Refusal> {
        let ids = edges_at(s, edges)?;
        blend::fillet_setback(s, &ids, radius, setback, "blend").map_err(refusal)
    }

    fn fillet_setback_g2(
        &self,
        s: &Brep,
        edges: &[model::Vec3],
        radius: f64,
        setback: f64,
    ) -> Result<Brep, Refusal> {
        let ids = edges_at(s, edges)?;
        blend::fillet_setback_g2(s, &ids, radius, setback, "blend").map_err(refusal)
    }

    fn affine(&self, s: &Brep, matrix: [[f64; 4]; 3]) -> Result<Brep, Refusal> {
        let a = edit::Affine {
            m: matrix.map(|r| [r[0], r[1], r[2]]),
            t: Vec3::new(matrix[0][3], matrix[1][3], matrix[2][3]),
        };
        edit::affine(s, &a).map_err(refusal)
    }

    fn chamfer_distances(
        &self,
        s: &Brep,
        edges: &[model::Vec3],
        distances: &[f64],
    ) -> Result<Brep, Refusal> {
        if edges.len() != distances.len() {
            return Err(Refusal::invalid(format!(
                "{} edges but {} sizes",
                edges.len(),
                distances.len()
            )));
        }
        let ids = edges_at(s, edges)?;
        let pairs: Vec<(EdgeId, f64)> = ids.into_iter().zip(distances.iter().copied()).collect();
        blend::chamfer_distances(s, &pairs, "blend").map_err(refusal)
    }

    fn chamfer_two(
        &self,
        s: &Brep,
        edge: model::Vec3,
        face: model::Vec3,
        d1: f64,
        d2: f64,
    ) -> Result<Brep, Refusal> {
        let edge = edges_at(s, &[edge])?.remove(0);
        blend::chamfer_two_edges(s, edge, face_at(s, face)?, d1, d2, "cha").map_err(refusal)
    }

    fn chamfer(&self, s: &Brep, edges: &[model::Vec3], distance: f64) -> Result<Brep, Refusal> {
        let ids = edges_at(s, edges)?;
        blend::chamfer_edges(s, &ids, distance, "cha").map_err(refusal)
    }

    fn transform(&self, s: &Brep, m: [[f64; 4]; 3]) -> Result<Brep, Refusal> {
        // A uniform scale times a rotation or a mirror; a stretch or shear is refused.
        let r = [0, 1, 2].map(|i| [m[i][0], m[i][1], m[i][2]]);
        let mut out = s.clone();
        out.transform_similar(r, Vec3::new(m[0][3], m[1][3], m[2][3]))
            .map_err(refusal)?;
        Ok(out)
    }

    fn raycast(&self, s: &Brep, o: model::Vec3, d: model::Vec3) -> Result<Vec<f64>, Refusal> {
        Ok(self.raycasts(s, &[(o, d)])?.remove(0))
    }

    fn raycasts(
        &self,
        s: &Brep,
        rays: &[(model::Vec3, model::Vec3)],
    ) -> Result<Vec<Vec<f64>>, Refusal> {
        if s.solid_ids().next().is_none() {
            return Ok(vec![Vec::new(); rays.len()]);
        }
        let caster = springrep::rays::Rays::new(s).map_err(refusal)?;
        rays.iter()
            .map(|&(o, d)| {
                let hits = caster.cast(v3(o), v3(d), (0.0, 1e9)).map_err(refusal)?;
                Ok(hits.iter().map(|h| h.t).collect())
            })
            .collect()
    }

    fn betti(&self, s: &Brep) -> Result<[u64; 3], Refusal> {
        Ok(measure::betti(s))
    }

    fn mesh(&self, s: &Brep, tolerance: f64) -> Result<brep_protocol::Mesh, Refusal> {
        let opt = springrep::mesh::MeshOptions::within(tolerance);
        let m = springrep::mesh::tessellate(s, &opt).map_err(refusal)?;
        Ok(brep_protocol::Mesh {
            positions: m.positions.iter().map(|p| [p.x, p.y, p.z]).collect(),
            triangles: m.triangles,
        })
    }

    fn display(&self, s: &Brep, tolerance: f64) -> Result<brep_protocol::Display, Refusal> {
        let d =
            springrep::mesh::display::display_within(s, tolerance, tolerance).map_err(refusal)?;
        let wide = |p: [f32; 3]| p.map(f64::from);
        Ok(brep_protocol::Display {
            faces: d
                .faces
                .into_iter()
                .map(|f| brep_protocol::DisplayFace {
                    positions: f.positions.into_iter().map(wide).collect(),
                    normals: f.normals.into_iter().map(wide).collect(),
                    triangles: f.triangles,
                })
                .collect(),
            edges: d
                .edges
                .iter()
                .map(|e| e.iter().map(|p| [p.x, p.y, p.z]).collect())
                .collect(),
        })
    }

    fn shell(&self, s: &Brep, open: &[model::Vec3], thickness: f64) -> Result<Brep, Refusal> {
        let open = faces_at(s, open)?;
        edit::shell(s, &open, thickness, edit::Wall::Inward, "shell").map_err(refusal)
    }

    fn draft(
        &self,
        s: &Brep,
        faces: &[model::Vec3],
        origin: model::Vec3,
        pull: model::Vec3,
        angle: f64,
    ) -> Result<Brep, Refusal> {
        let faces: Vec<FaceId> = faces
            .iter()
            .map(|&p| face_at(s, p))
            .collect::<Result<_, _>>()?;
        let z = v3(pull);
        let hint = if z.x.abs() < 0.9 {
            Vec3::new(1.0, 0.0, 0.0)
        } else {
            Vec3::new(0.0, 1.0, 0.0)
        };
        let neutral = Frame::new(v3(origin), z, hint).map_err(refusal)?;
        edit::draft(s, &faces, &neutral, angle).map_err(refusal)
    }

    fn write_step(&self, s: &Brep) -> Result<String, Refusal> {
        springrep::step::write(s, "conformance").map_err(refusal)
    }

    fn read_step(&self, text: &str) -> Result<Brep, Refusal> {
        springrep::step::read_part(text.as_bytes(), "step").map_err(refusal)
    }

    fn centroid(&self, s: &Brep) -> Result<Option<model::Vec3>, Refusal> {
        if s.solid_ids().next().is_none() {
            return Ok(None);
        }
        let c = measure::mass_properties(s).centre;
        Ok(Some([c.x, c.y, c.z]))
    }

    fn inertia(&self, s: &Brep) -> Result<[[f64; 3]; 3], Refusal> {
        if s.solid_ids().next().is_none() {
            return Ok([[0.0; 3]; 3]);
        }
        Ok(measure::mass_properties(s).inertia)
    }

    fn loft_through(
        &self,
        frame: &model::Frame,
        profiles: &[profile::Profile],
        heights: &[f64],
    ) -> Result<Brep, Refusal> {
        let e = refusal;
        if profiles.len() != heights.len() {
            return Err(Refusal::invalid(format!(
                "{} profiles at {} heights",
                profiles.len(),
                heights.len()
            )));
        }
        let sections = profiles
            .iter()
            .zip(heights)
            .map(|(p, &h)| {
                let at = v3(frame.origin) + v3(frame.z) * h;
                Ok(Section {
                    profile: sketch(p)?,
                    frame: Frame::new(at, v3(frame.z), v3(frame.x)).map_err(e)?,
                })
            })
            .collect::<Result<Vec<_>, Refusal>>()?;
        loft(&sections, false, "loft").map_err(e)
    }

    fn normal(
        &self,
        s: &Brep,
        points: &[model::Vec3],
    ) -> Result<Vec<Option<model::Vec3>>, Refusal> {
        let outlines = measure::Outlines::new(s);
        Ok(points
            .iter()
            .map(|&p| {
                outlines
                    .normal_at(v3(p), 10.0 * tol::LINEAR)
                    .map(Vec3::to_array)
            })
            .collect())
    }

    fn interference(&self, a: &Brep, b: &Brep) -> Result<brep_protocol::Meeting, Refusal> {
        let (volume, gap) = match overlap::between(a, b).map_err(refusal)? {
            overlap::Meeting::Shared(v) => (v, 0.0),
            overlap::Meeting::Touching => (0.0, 0.0),
            overlap::Meeting::Apart(d) => (0.0, d),
        };
        Ok(brep_protocol::Meeting { volume, gap })
    }

    fn offset(&self, s: &Brep, distance: f64) -> Result<Brep, Refusal> {
        edit::offset(s, distance, "offset").map_err(refusal)
    }

    fn thicken(&self, s: &Brep, faces: &[model::Vec3], thickness: f64) -> Result<Brep, Refusal> {
        let faces = faces_at(s, faces)?;
        edit::thicken(s, &faces, thickness).map_err(refusal)
    }

    fn delete_faces(&self, s: &Brep, faces: &[model::Vec3]) -> Result<Brep, Refusal> {
        let faces = faces_at(s, faces)?;
        edit::delete_faces(s, &faces).map_err(refusal)
    }

    fn offset_faces(
        &self,
        s: &Brep,
        faces: &[model::Vec3],
        distance: f64,
    ) -> Result<Brep, Refusal> {
        let by: HashMap<FaceId, f64> = faces
            .iter()
            .map(|&p| face_at(s, p).map(|f| (f, distance)))
            .collect::<Result<_, _>>()?;
        edit::offset_faces(s, &by).map_err(refusal)
    }

    fn move_faces(
        &self,
        s: &Brep,
        faces: &[model::Vec3],
        matrix: [[f64; 4]; 3],
    ) -> Result<Brep, Refusal> {
        let t = Transform::rigid(
            [0, 1, 2].map(|i| [matrix[i][0], matrix[i][1], matrix[i][2]]),
            Vec3::new(matrix[0][3], matrix[1][3], matrix[2][3]),
        )
        .map_err(refusal)?;
        let faces = faces_at(s, faces)?;
        edit::move_faces_by(s, &faces, &t).map_err(refusal)
    }

    fn split(
        &self,
        s: &Brep,
        origin: model::Vec3,
        normal: model::Vec3,
    ) -> Result<(Brep, Brep), Refusal> {
        boolean::split(s, v3(origin), v3(normal)).map_err(refusal)
    }

    fn section(
        &self,
        s: &Brep,
        origin: model::Vec3,
        normal: model::Vec3,
        tolerance: f64,
    ) -> Result<Vec<Vec<model::Vec3>>, Refusal> {
        let curves = measure::section(s, v3(origin), v3(normal), tolerance).map_err(refusal)?;
        // Each of the kernel's curves is closed, and one piece: it ends on its first point.
        Ok(curves
            .into_iter()
            .map(|c| c.iter().chain(c.first()).map(|p| p.to_array()).collect())
            .collect())
    }

    fn section_area(
        &self,
        s: &Brep,
        origin: model::Vec3,
        normal: model::Vec3,
    ) -> Result<f64, Refusal> {
        measure::section_area(s, v3(origin), v3(normal)).map_err(refusal)
    }

    fn fillet_g2(
        &self,
        s: &Brep,
        edges: &[model::Vec3],
        offset: f64,
        center: f64,
    ) -> Result<Brep, Refusal> {
        let edges = edges_at(s, edges)?;
        blend::fillet_g2(s, &edges, offset, center, "blend").map_err(refusal)
    }

    fn fillet_variable(
        &self,
        s: &Brep,
        edge: model::Vec3,
        from: model::Vec3,
        r_from: f64,
        r_to: f64,
    ) -> Result<Brep, Refusal> {
        let e = edges_at(s, &[edge])?[0];
        blend::fillet_variable(s, e, v3(from), r_from, r_to, "blend").map_err(refusal)
    }

    fn fillet_conic(
        &self,
        s: &Brep,
        edges: &[model::Vec3],
        setback: f64,
        rho: f64,
    ) -> Result<Brep, Refusal> {
        let edges = edges_at(s, edges)?;
        blend::fillet_conic(s, &edges, setback, rho, "blend").map_err(refusal)
    }

    fn fillet_faces(
        &self,
        s: &Brep,
        face_a: model::Vec3,
        face_b: model::Vec3,
        radius: f64,
    ) -> Result<Brep, Refusal> {
        let (a, b) = (face_at(s, face_a)?, face_at(s, face_b)?);
        blend::fillet_faces(s, a, b, radius, "blend").map_err(refusal)
    }

    fn full_round(
        &self,
        s: &Brep,
        side_a: model::Vec3,
        middle: model::Vec3,
        side_b: model::Vec3,
    ) -> Result<Brep, Refusal> {
        let (a, m, b) = (
            face_at(s, side_a)?,
            face_at(s, middle)?,
            face_at(s, side_b)?,
        );
        blend::full_round(s, a, m, b, "blend").map_err(refusal)
    }

    fn project(
        &self,
        s: &Brep,
        direction: model::Vec3,
        tolerance: f64,
    ) -> Result<brep_protocol::Projection, Refusal> {
        let drawn = springrep::draw::hidden_lines(s, v3(direction), tolerance).map_err(refusal)?;
        let (u, v) = brep_protocol::view_axes(direction);
        let (u, v) = (v3(u), v3(v));
        let flat = |lines: Vec<Vec<Vec3>>| {
            lines
                .into_iter()
                .map(|l| l.iter().map(|p| [p.dot(u), p.dot(v)]).collect())
                .collect()
        };
        Ok(brep_protocol::Projection {
            visible: flat(drawn.visible),
            hidden: flat(drawn.hidden),
        })
    }

    fn curvature(
        &self,
        s: &Brep,
        points: &[model::Vec3],
    ) -> Result<Vec<Option<[f64; 2]>>, Refusal> {
        let outlines = measure::Outlines::new(s);
        Ok(points.iter().map(|&p| outlines.curvature(v3(p))).collect())
    }

    fn sew(&self, text: &str, tolerance: f64) -> Result<Brep, Refusal> {
        springrep::step::sew(text.as_bytes(), tolerance).map_err(refusal)
    }

    fn bounding_box(&self, s: &Brep) -> Result<[model::Vec3; 2], Refusal> {
        let b = measure::bbox(s);
        Ok([b.min.to_array(), b.max.to_array()])
    }

    fn distance(&self, a: &Brep, b: &Brep) -> Result<f64, Refusal> {
        distance::closest(a, &Selection::body(a), b, &Selection::body(b))
            .map(|c| c.distance)
            .map_err(refusal)
    }
}

fn main() -> ExitCode {
    match brep_protocol::serve(&Springrep) {
        Ok(()) => ExitCode::SUCCESS,
        Err(e) => {
            eprintln!("{e}");
            ExitCode::FAILURE
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use brep_protocol::Display;

    /// The `ASSET_GENERATOR` the published shapes were drawn under, and the fingerprint of what
    /// springrep's `display` drew then.
    const DRAWN_UNDER: (&str, u64) = ("4", 0x62ae86b57aef1ddc);

    fn fnv(h: &mut u64, x: i64) {
        for b in x.to_le_bytes() {
            *h = (*h ^ u64::from(b)).wrapping_mul(0x100_0000_01b3);
        }
    }

    /// Each face's counts, area, centroid and mean normal, and each edge's length, rounded coarsely
    /// enough that another platform's last-bit differences in trigonometry do not move them.
    fn fingerprint(h: &mut u64, d: &Display) {
        let round = |x: f64| (x * 1e3).round() as i64;
        fnv(h, d.faces.len() as i64);
        for f in &d.faces {
            fnv(h, f.positions.len() as i64);
            fnv(h, f.triangles.len() as i64);
            let (mut area, mut mid, mut normal) = (0.0, [0.0; 3], [0.0; 3]);
            for t in &f.triangles {
                let [p, q, r] = t.map(|i| f.positions[i as usize]);
                let (u, v) = (
                    [0, 1, 2].map(|k| q[k] - p[k]),
                    [0, 1, 2].map(|k| r[k] - p[k]),
                );
                let n = [
                    u[1] * v[2] - u[2] * v[1],
                    u[2] * v[0] - u[0] * v[2],
                    u[0] * v[1] - u[1] * v[0],
                ];
                let a = (n[0] * n[0] + n[1] * n[1] + n[2] * n[2]).sqrt() / 2.0;
                area += a;
                for k in 0..3 {
                    mid[k] += a * (p[k] + q[k] + r[k]) / 3.0;
                }
            }
            for n in &f.normals {
                for k in 0..3 {
                    normal[k] += n[k] / f.normals.len() as f64;
                }
            }
            fnv(h, round(area));
            for k in 0..3 {
                fnv(h, round(mid[k] / area.max(1e-12)));
                fnv(h, round(normal[k]));
            }
        }
        fnv(h, d.edges.len() as i64);
        for e in &d.edges {
            fnv(h, e.len() as i64);
            let length: f64 = e
                .windows(2)
                .map(|w| {
                    (0..3)
                        .map(|k| (w[1][k] - w[0][k]).powi(2))
                        .sum::<f64>()
                        .sqrt()
                })
                .sum();
            fnv(h, round(length));
        }
    }

    #[test]
    fn the_published_shapes_are_still_what_display_draws() {
        let k = Springrep;
        let at = model::Frame::at([1.0, 2.0, 3.0]);
        let make = |p: Primitive| k.make(&p).unwrap();
        let block = make(Primitive::Box {
            frame: at,
            size: [10.0, 20.0, 30.0],
        });
        let rod = make(Primitive::Cylinder {
            frame: model::Frame::at([6.0, 12.0, 0.0]),
            radius: 3.0,
            height: 40.0,
        });
        let ball = make(Primitive::Sphere {
            frame: at,
            radius: 8.0,
        });
        let drilled = k.difference(&block, &rod).unwrap();
        let mut h = 0xcbf2_9ce4_8422_2325;
        for s in [&block, &rod, &ball, &drilled] {
            for tolerance in [0.02, 0.005] {
                fingerprint(&mut h, &k.display(s, tolerance).unwrap());
            }
        }
        let now = (brep_conformance::run::ASSET_GENERATOR, h);
        assert_eq!(
            now, DRAWN_UNDER,
            "springrep's display draws differently, so the published GLBs no longer match it: \
             bump ASSET_GENERATOR in brep-conformance's suite/src/run.rs, so the next publish \
             names new files, and set DRAWN_UNDER to the new pair"
        );
    }
}
