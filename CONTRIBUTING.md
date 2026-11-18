# Contributing

Issues and pull requests are welcome. A Springcraft maintainer reviews every pull request, because
the suite's results come only from commits reviewed as pass-through.

## What belongs here

- **Corrections.** A place where the adapter drives springrep wrongly, or calls something other than
  the kernel's one call for a function.
- **New calls.** A function the adapter lists as unsupported that a newer springrep release now
  offers in one call.

What does not belong: work the kernel does not do itself. The adapter is pass-through, as the
suite's
[contract](https://github.com/springcraft-ai/brep-conformance/blob/main/doc/contract.md#what-an-adapter-may-do)
asks. A function assembled here out of several kernel calls would score this code and not springrep,
so it stays unsupported.

A problem with a test, a check or the protocol belongs in
[brep-conformance](https://github.com/springcraft-ai/brep-conformance/issues). A problem in
springrep itself belongs with springrep.

## Licence of contributions

Contributions are accepted under this repository's [MIT licence](LICENSE).

springrep has a licence of its own, which the [README](README.md#licences) describes. Call the
kernel; do not copy its source, or any other project's, into this repository.
