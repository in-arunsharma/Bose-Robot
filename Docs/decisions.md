# Design & Engineering Decisions — Bose Stair-Climbing Robot

This document contains a curated list of key decisions that significantly impact the design, architecture, and performance of the Bose robot project.  
Each decision is recorded in a structured, unambiguous format to ensure traceability, technical clarity, and reproducibility of the project workflow.

---

## 2025-05-03 — Standardized Format for `decisions.md`

**Topic:** Documentation Structure  
**Decision:** Use a structured Markdown format for `decisions.md`, with one entry per decision including:  
- Topic  
- Date  
- Final decision  
- Technical rationale  
- Alternatives considered (if any)  
- Links to relevant logs or experiments  

---

### Context

During the first phase of the Bose robot project, multiple technical decisions were embedded inside daily progress logs (`/docs/logs/log_yyyy-mm-dd.md`). While the logs were valuable for tracking work sessions, they mixed up trials, team presence, tests, and final outcomes — making it **difficult to identify confirmed project-wide decisions**.

---

### Chosen Format

We decided to create and maintain a separate file, `decisions.md`, using the following standardized entry structure:

```markdown
## YYYY-MM-DD — [Clear, Specific Title]

**Topic:** [Component, concept, or design area]  
**Decision:** [Final choice made — objective and unambiguous]  
**Rationale:**  
- [Technical reason 1]  
- [Technical reason 2]  
- [Relevant constraints considered: cost, compatibility, power, etc.]  
**Alternatives Considered:**  
- [Alternative 1 — why rejected]  
- [Alternative 2 — why rejected]  
**Related Logs:** [`log_YYYY-MM-DD.md`](./logs/log_YYYY-MM-DD.md)
