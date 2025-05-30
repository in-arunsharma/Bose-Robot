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
- Alternatives considered  
- Links to relevant logs or tests  

**Rationale:**  
During the first phase of the Bose robot project, multiple technical decisions were embedded inside daily progress logs (`/docs/logs/log_yyyy-mm-dd.md`). While those logs were valuable for tracking progress, they mixed together experiments, tests, and conclusions — making it difficult to identify and trace final, project-wide decisions.  
This format separates final decisions for clarity, traceability, and report integration.  

**Alternatives Considered:**  
- Embedding decisions in daily logs → too verbose and hard to trace  
- GitHub Issues → overkill for the scope of the project  
- Wiki page → lacks version control and log linkage  

**Chosen Format:**  
Every decision entry in this file must follow this structure:

```markdown
## YYYY-MM-DD — [Clear, Specific Title]

**Topic:** [Component, concept, or design area]  
**Decision:** [Final choice made — objective and unambiguous]  
**Rationale:**  
- [Technical reason 1]  
- [Technical reason 2]  
- [Constraints: cost, size, compatibility, power, etc.]  
**Alternatives Considered:**  
- [Alternative 1 — why rejected]  
- [Alternative 2 — why rejected]  
**Related Logs:** [`log_YYYY-MM-DD.md`](./logs/log_YYYY-MM-DD.md)
```

---
