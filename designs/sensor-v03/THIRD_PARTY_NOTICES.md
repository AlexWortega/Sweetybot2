# Inspiration and third-party material

**Inspired by [Sweetie Bot](https://github.com/sweetie-bot-project/sweetie_bot) and [Microduck](https://github.com/pollen-robotics/microduck).** This is an independent experimental redesign; neither project endorses it.

- Sweetie Bot Project: pony appearance, expressive head/ears and reference control architecture. The saved Sweetie Bot software reference retains its BSD notice in `references/upstream/sweetie_bot/LICENSE`.
- Pollen Robotics Microduck: compact smart-servo architecture, maintainability and hardware-aware simulation. The saved software reference retains its Apache-2.0 text in `references/upstream/microduck/LICENSE`. Hardware/model rights are separate from software rights; Microduck CAD and trained weights were not copied into this design.
- Microduck RL: research/reference configuration, with its original notice in `references/upstream/microduck_rl/LICENSE`. Its biped weights are not used here.
- [text-to-cad](https://github.com/earthtojake/text-to-cad) / cadgen: CAD authoring toolchain. This repository contains project-authored parametric factories, not a vendored copy of that toolchain.
- Catalog models from [step.parts](https://www.step.parts/): STS3250 servo, comparator servo, bearing and screw. Exact URLs and SHA-256 values are retained in `references/catalog_parts.json`. Vendor/product material retains its original ownership; inclusion does not relicense it.
- Sweetie Bot reference photos and manufacturer documents retain their original ownership and source attribution in `references/appearance/manifest.json` and `docs/provenance.md`.

Pinned revisions and reference-file hashes are in `references/upstream/manifest.json`. Dependencies and third-party reference files retain their respective licenses. No blanket license has been selected for the newly authored project files; publication does not change third-party rights.
