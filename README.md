# brep-conformance-springrep

The [brep-conformance](https://github.com/springcraft-ai/brep-conformance) adapter for
[springrep](https://github.com/springcraft-ai/springrep): it puts springrep behind the suite's
protocol, so that the suite can score it.

## Who maintains this

brep-conformance scores a B-rep kernel through an adapter: a small program that answers the suite's
requests with the kernel's own calls. Springcraft maintains the suite, springrep and this adapter.

- **It adds nothing to the kernel.** The adapter is pass-through, as the suite's
  [contract](https://github.com/springcraft-ai/brep-conformance/blob/main/doc/contract.md#what-an-adapter-may-do)
  asks: one call of the kernel for each function, and anything the kernel has no one call for is
  listed as unsupported. What is scored is springrep, not this code.
- **The suite decides what is scored.** An official run uses only a commit of this adapter that the
  suite's maintainers have reviewed as pass-through.

## Overview

The adapter answers the requests of the suite's
[api.md](https://github.com/springcraft-ai/brep-conformance/blob/main/doc/api.md#calling-a-kernel)
on stdin and stdout, through `serve` in the `brep-protocol` crate. [`src/main.rs`](src/main.rs)
answers each function with this kernel's one call for it, as
[the contract](https://github.com/springcraft-ai/brep-conformance/blob/main/doc/contract.md#what-an-adapter-may-do)
asks, and lists nothing as unsupported. What it adds is translation: the suite's profiles and frames
as the kernel's, a point as the edge or face it lies on through `measure::edge_at` and
`measure::face_at` at `10·ε`, and the kernel's errors as the protocol's refusals, each with the
kernel's own kind. A STEP file is read as one part, `step::read_part`: a result whose pieces touch
is written as one solid per piece, and reads back as their union.

## Setup

The Rust toolchain [`rust-toolchain.toml`](rust-toolchain.toml) pins.

## Usage

The suite runs the adapter as a separate process. Build the adapter here, then run the suite from a
checkout of [brep-conformance](https://github.com/springcraft-ai/brep-conformance) beside this one:

```bash
cargo build --release
cd ../brep-conformance
cargo build --release -p brep-conformance
target/release/brep-conformance --adapter ../brep-conformance-springrep/target/release/springrep-adapter \
    --out springrep.json
```

## Licences

The adapter and springrep are both under the MIT licence: see [LICENSE](LICENSE) here and
springrep's own repository.

## Contributing

See [CONTRIBUTING.md](CONTRIBUTING.md).
