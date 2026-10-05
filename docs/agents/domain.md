# Domain Docs

How engineering skills consume this repository’s domain documentation.

## Before exploring, read these

- `CONTEXT.md` at the repository root.
- `docs/adr/` entries relevant to the area being changed.

If these files do not exist, proceed silently. The domain-modeling skill creates them lazily when terms or decisions are resolved.

## File structure

This repository uses a single domain context:

```
/
├── CONTEXT.md
├── docs/
│   └── adr/
└── src/
```

`CONTEXT.md` is strictly a glossary and contains no implementation details. ADRs record architectural decisions that are hard to reverse, surprising without context, and based on a real trade-off.

## Use the glossary’s vocabulary

Use terms as defined in `CONTEXT.md` in issue titles, proposals, tests, and implementation discussions. Avoid synonyms that the glossary explicitly rejects.

If a needed concept is absent, reconsider whether it belongs to the domain language or note the gap for domain modeling.

## Flag ADR conflicts

If proposed work contradicts an existing ADR, surface the conflict explicitly rather than silently overriding it.
