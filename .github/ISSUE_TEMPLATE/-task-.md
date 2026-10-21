---
name: "[TASK]"
about: Use this when creating tasks.
title: ''
labels: ''
assignees: ''
type: TASK

---

<!--
HOW TO USE THIS TEMPLATE
- Fill it in top to bottom and replace every <placeholder>.
- If a line doesn't apply, write "N/A — <reason>" instead of deleting it.
- Title the issue with a verb and an object, e.g. "Add IMU noise to the simulation".
- Parent epic, assignee, status, priority, estimate, dates, and blocking issues are set in GitHub Projects, not here.
- Ask questions in the issue comments and raise them at scrum.
- Comments like this one are hidden when the issue is viewed, so you can leave them in.
-->
 
## 1. Task
 
<!-- After reading this section, someone new to the team should understand the work and why it matters. -->
 
- **What:** <1–3 sentences describing the work>
- **Why:** <What this enables or unblocks for the parent epic>
- **Covers:** <Epic requirement IDs, e.g. R1, R3>
- **Out of scope:** <What this task deliberately leaves out, or None>
## 2. Done When
 
<!-- The only place this task's success is defined. Each line is one pass/fail result and how it will be checked:
Test = measured · Demonstration = shown working · Analysis = calculation, simulation, or research · Inspection = checked by review -->
 
- [ ] <Expected result> **<Test / Demonstration / Analysis / Inspection>:** <How it will be checked>
- [ ] <Expected result>  **<Test / Demonstration / Analysis / Inspection>:** <How it will be checked>
## 3. Getting Started
 
<!-- What someone needs before starting that isn't already a linked or blocking issue. -->
 
- **Prerequisites:** <Equipment, configuration, access, or files needed, or None>
- **Starting points:** <Relevant code, files, designs, or people to ask>
- **Notion:** <Link to the relevant documentation section>
## 4. Verification Record
 
<!-- Fill in when the work is complete. -->
 
- **Evidence:** <For each Done When item: the result, with a link to test output, photos, or video>
- **Tested version:** <Exact Git commit tested>
## 5. Definition of Done
 
<!-- The same for every task. Only change a line if it truly doesn't apply. -->
 
- [ ] Every Done When item is met, with evidence in Section 4.
- [ ] The work matches the parent epic's interfaces.
- [ ] Documentation is updated according to the Documentation SOP.
- [ ] Limitations and follow-up work are recorded as new issues.
## 6. When Done
 
<!-- The same for every task. Follow these steps in order once Section 5 is complete. -->
 
1. Open a pull request from the task branch into the epic branch, and link it to this issue.
2. Get the pull request peer reviewed and approved.
3. Confirm all tests pass.
4. Merge the pull request into the epic branch and delete the task branch.
5. Close this issue by hand. GitHub only closes linked issues automatically when a pull request merges into the default branch, not an epic branch.
