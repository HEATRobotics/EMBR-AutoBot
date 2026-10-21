---
name: "[EPIC]"
about: Use this when creating epics.
title: "[EPIC]"
labels: ''
assignees: maisonJar-dev
type: EPIC

---

# [EPIC] <Epic Name>
 
<!--
HOW TO USE THIS TEMPLATE
- Fill it in top to bottom and replace every <placeholder>.
- If a line doesn't apply, write "N/A — <reason>" instead of deleting it.
- Keep to one line per item where possible. Long explanations go in the Notion document and are linked here.
- Status, dates, blocking, and sub-issues are set in GitHub Projects, not here. Priority and estimates go on the sub-issues, not the epic.
- Ask questions in the issue comments and raise them at scrum.
- Comments like this one are hidden when the issue is viewed, so you can leave them in.
-->
 
| Field | Value |
| --- | --- |
| **Notion Document** | <Link> |
 
## 1. Overview
 
<!-- After reading this section, someone outside the team should understand why this epic exists. -->
 
- **Summary:** <2–3 sentences on what this epic delivers and why it matters>
- **Current state:** <What exists or works today>
- **Problem:** <What is missing, broken, or blocking us>
- **Impact:** <Who or what is affected if this isn't solved>
## 2. Definition of Success
 
<!-- The only place this epic's success is defined. State the MVP, list 3–6 pass/fail checks that prove it, then describe how it will be shown working.
Hardware epics usually include safety checks, e.g. "Actuation stays disabled until armed" or "Loss of communication stops motion within <time>". -->
 
**MVP:** <1–3 sentences on the smallest useful version this epic must deliver>
 
**Success criteria:**
 
- [ ] <Observable pass/fail outcome, with a value and units where possible>
- [ ] <Observable pass/fail outcome>
- [ ] <Observable pass/fail outcome>
**Demonstration:** <What will be shown working, where (simulation, test fixture, or robot), and under what conditions. Note anything it does not prove.>
 
## 3. Scope
 
<!-- Be explicit about what is out. Every deferred item points to a follow-up issue. -->
 
**In scope:**
 
- <Item>
- <Item>
**Out of scope / Deferred:**
 
- <Item> → <Follow-up issue>
**Constraints:** <Limits the work must respect, or things this epic must not depend on>
 
**Assumptions:** <What we are taking as true; flag any that need confirming>
 
## 4. Requirements
 
<!-- What must be built or true to meet the success criteria. Each requirement gets an ID and a sub-issue, and tasks refer to it by ID.
Verification: Test = measured · Demonstration = shown working · Analysis = calculation, simulation, or research · Inspection = checked by review -->
 
| ID | Requirement | Verification | Sub-issue |
| --- | --- | --- | --- |
| R1 | <What must be built or true> | <Test / Demonstration / Analysis / Inspection> | <#issue> |
| R2 | <What must be built or true> | <Test / Demonstration / Analysis / Inspection> | <#issue> |
 
## 5. Deliverables
 
<!-- Concrete things that will exist when the epic is done, and where each lives, e.g. code, firmware, documents, configurations, hardware. -->
 
- [ ] <Deliverable> (<repo / Notion / physical>)
- [ ] <Deliverable> (<repo / Notion / physical>)
## 6. Interfaces
 
<!-- What this epic connects to, and what is agreed for each connection. -->
 
| Connection | What is exchanged | Agreement |
| --- | --- | --- |
| <This epic> ↔ <System, team, or epic> | <Data, messages, documents, or requirements> | <Format, units, rate, or other agreement> |
 
## 7. Dependencies
 
<!-- What must be ready from other people or teams. Blocking epics are set in GitHub Projects; list one here only to say exactly what is needed from it. -->
 
| Dependency | What we need | Owner |
| --- | --- | --- |
| <Item, team, or resource> | <What must be available> | <Person or team> |
 
## 8. Risks
 
<!-- Specific risks for this epic: what could happen, what it would affect, and how it is handled.
Common risks: integration mismatches, missing inputs from other teams, unsafe test conditions, tools or resources not available. -->
 
| Risk | Description | Mitigation | Owner |
| --- | --- | --- | --- |
| <Short name> | <What could happen, and what it would affect> | <How it is prevented, detected, or handled> | <Person> |
 
**Safe response during testing:** <e.g. e-stop, cut power, support the load>
 
## 9. Documentation
 
<!-- Links only; the detail lives in the linked pages. Fill in evidence and versions once the work is validated. -->
 
- **Design / CAD / Schematics:** <Links>
- **Datasheets / Technical documentation:** <Links>
- **Demonstration and test evidence:** <Notion link to results, photos, or video>
- **Tested version:** <Exact Git commit tested>
- **Merged version:** <Link to the annotated Git tag>
- **Documentation SOP:** <Link>
## 10. Definition of Done
 
<!-- The same for every epic. Only change a line if it truly doesn't apply. -->
 
- [ ] Every success criterion in Section 2 is met, and the demonstration evidence is linked in Section 9.
- [ ] Every requirement passes its verification, or the epic lead accepts the gap and it is recorded as a follow-up issue.
- [ ] Interfaces are confirmed with the connected systems or teams.
- [ ] Changes between the tested and merged versions are checked, and affected tests repeated.
- [ ] Documentation follows the SOP and lets a new member understand and use the deliverable.
- [ ] Remaining limitations are recorded as follow-up issues.
## 11. When Done
 
<!-- The same for every epic. Follow these steps in order once Section 10 is complete. -->
 
1. Open a pull request from the epic branch into main, with "Closes #<this issue>" in its description.
2. Get the pull request peer reviewed and approved by the current Software Executive and another related member.
3. Confirm all tests pass.
4. Merge the pull request into main and delete the epic branch. This issue closes automatically.
5. Create an annotated Git tag on the merge commit, push it to GitHub, and link it in Section 9.
6. Add the tag and a one-line summary of what was delivered to the Notion document.
