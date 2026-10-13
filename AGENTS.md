# AGENTS.md

Guidance for AI coding agents (and humans) working in this repo. Read `README.md`
first: architecture (core/shell separation), the JSON contracts, CI gates, and the
bit-level numerical reproducibility rules are non-negotiable.

Build/test as usual: `cargo build --workspace` / `cargo test --workspace`.

## Product-direction decision records

### PDR-1 — Onshape is the first-choice CAD platform for future integration
*(recorded 2026-09-14; web research from that date — re-verify specifics before acting)*

**Decision.** When Ansatz grows an integration/interop layer (assembly import/export,
agent-driven cloud modeling, live solving against a host CAD), **prioritize Onshape
first**: cloud-native REST API (+ websockets), fully constraint-driven sketches and
assemblies. It is the "player #1" candidate from our platform survey.

**Verified facts:**

- Onshape's geometry stack is **not in-house**. Kernel = **Parasolid**, constraint
  solver = **D-Cubed (DCM family)** — both licensed from Siemens PLM. Onshape's own
  engineering post (Michael Lauer, 2018-06-28): "…the venerable geometric modeling
  kernel Parasolid and constraint solver D-Cubed … which we license from Siemens PLM."
  Both sketch solving and assembly-instance positioning go through D-Cubed (the post
  does not split 2D DCM vs 3D DCM).
- What Onshape actually built in-house is the **platform**: multi-user database-style
  data management, custom renderers, their own wire protocol, FeatureScript. That is
  where its API value lives — the constraint semantics underneath are Siemens parts.
- Ownership lineage (both components are Cambridge, UK kin): Parasolid — Shape Data →
  Evans & Sutherland → McDonnell Douglas (1988, with Unigraphics) → EDS (1991) →
  private equity / UGS Corp (2004) → Siemens (2007, ≈US$3.5B for UGS). D-Cubed —
  founded 1989 by John Owen (ex-Shape Data) → acquired by UGS (June 2004) → Siemens
  (2007). Both now sold under Siemens Digital Industries Software "PLM Components".
- PTC acquired Onshape (announced 2019-10-23, closed Nov 2019, ≈US$470M net of
  cash). **No kernel or solver switch since**: Onshape still runs Parasolid + D-Cubed;
  PTC's Creo runs its own Granite kernel, so Onshape↔Creo still requires geometry
  translation. PTC keeps paying license fees to its competitor Siemens for Onshape.
- SolidWorks (Onshape founders' previous company) uses the same Parasolid + DCM
  combination; the DCM/Parasolid duopoly is the industry default.

**Implications for Ansatz:**

- Onshape's API semantics ride on a stable Parasolid + DCM base — low drift risk from
  corporate-ownership changes. Safe long-horizon integration target.
- DCM/Parasolid are not realistically licensable by a small team (Siemens PLM
  Components is enterprise-scale negotiation). Our references stay: SolveSpace and
  FreeCAD planegcs (open), LEDAS LGS (commercial ceiling) — see README
  Acknowledgements.
- Treat Onshape's mate/feature semantics as *their wrapper over DCM*, not as ground
  truth for our own solver semantics. Our differentiation stays: diagnostics-first,
  structured DOF/redundancy reporting, embed-anywhere single core.

**Sources** (all accessed 2026-09-14):

- [Onshape — How Does Onshape Really Work?](https://www.onshape.com/en/blog/how-does-onshape-really-work)
- [engineering.com — Parasolid, D-Cubed and Siemens](https://www.engineering.com/parasolid-d-cubed-and-siemens-the-heart-of-your-cad-software-belongs-to-another/)
- [Upfront e-Zine (R. Grabowski) — Onshape Parasolid vs Creo Granite](https://upfrontezine.substack.com/p/why-cad-haz-proprietary-formats)
- [PTC press release — acquisition of Onshape](https://www.ptc.com/en/news/2019/ptc-to-acquire-leading-saas-product-development-platform-provider-onshape)
- [Reuters — Siemens to buy UGS for $3.5B](https://www.reuters.com/article/business/siemens-to-buy-ugs-for-35-billion-plans-ipo-of-vdo-idUSN24456582/)
- [isicad — UGS acquires D-Cubed (2004)](http://isicad.net/news.php?news=10034)
- [Wikipedia — Shape Data](https://en.wikipedia.org/wiki/Shape_Data) / [UGS Corp.](https://en.wikipedia.org/wiki/UGS_Corp.)
