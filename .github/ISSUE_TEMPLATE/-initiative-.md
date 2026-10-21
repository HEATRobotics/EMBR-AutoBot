---
name: "[INITIATIVE]"
about: Use this when creating initiatives.
title: ''
labels: ''
assignees: maisonJar-dev
type: INITIATIVE

---

# [INITIATIVE] <Initiative Name>
 
<!--
HOW TO USE THIS TEMPLATE
- Fill it in top to bottom and replace every <placeholder>.
- If a line doesn't apply, write "N/A — <reason>" instead of deleting it.
- Keep to one line per item where possible. Long explanations go in the Notion document and are linked here.
- Status, target date, blocking, and epics (as sub-issues) are set in GitHub Projects, not here.
- Ask questions in the issue comments and raise them at scrum.
- Comments like this one are hidden when the issue is viewed, so you can leave them in.
-->
 
| Field | Value |
| --- | --- |
| **Initiative Lead** | <@name> |
| **Notion Document** | <Link> |
 
## 1. Overview
 
<!-- After reading this section, someone outside the team should understand why this initiative exists. -->
 
- **Summary:** <2–3 sentences on the EMBR subsystem and the capability the robot will have once this initiative is complete>
- **Current state:** <What exists or works today>
- **Problem:** <What is missing, broken, or blocking us>
- **Impact:** <Who or what is affected if this isn't solved>
## 2. Goals
 
<!-- The capabilities this initiative is working toward. Which of them count toward success is decided in Section 3. -->
 
- **<Capability>:** <Observable outcome on the robot>
- **<Capability>:** <Observable outcome on the robot>
- **Operating constraints:** <Allowed modes, sequencing rules (e.g. one motion mode at a time), limits, and operating conditions>
- **Physical feedback:** <What final physical state is known, and whether it is measured or estimated; accuracy and reference strategy, or TBD>
- **Stretch goals:** <Capabilities worth attempting beyond the MVP, e.g. simultaneous operation, or None>
## 3. Definition of Success
 
<!-- The only place this initiative's success is defined. State the MVP, list the pass/fail checks that prove it, then describe how it will be shown working.
Resolve every <value> before the affected work is marked Ready.
Simulation and controller-logic checks belong in the epics' requirements; the demonstration here shows the integrated system. -->
 
**MVP:** <1–3 sentences on the smallest useful version: which goals from Section 2 it must deliver>
 
**Success criteria:**
 
- [ ] Startup is repeatable, verifies calibration, feedback, and a known reference, and prevents actuation until explicitly armed.
- [ ] Startup, shutdown, and fault recovery cause no unintended motion.
- [ ] All <number> actuators/axes are individually addressable and mapped to the correct physical component.
- [ ] <Operating and travel> limits are documented and enforced.
- [ ] <Operating constraint, e.g. one motion mode at a time>; conflicting requests are rejected or queued according to a documented rule.
- [ ] Targets are reached within <tolerance and units>, within <time>, under <load and support conditions>.
- [ ] Feedback accuracy is verified against <independent measurement or reference> within <tolerance>.
- [ ] Loss of commands, feedback, or one controller triggers the documented response within <time> and prevents <uncontrolled motion, descent, or load release>.
**Demonstration:** Given that <hardware is set up, all controllers/devices communicate over <protocol>, and feedback loops are running>, the robot can <behaviour>, under the operating constraints in Section 2. Show startup, controlled operation, shutdown, and the documented fault responses.
 
**If the robot is unavailable:** <Approved test fixture: its setup, representative load and geometry, what it proves, and what it does not>. Approver: <@name>
 
## 4. Epics
 
<!-- Each epic delivers one independently testable capability. Create each from the EPIC template and add it as a sub-issue; its lead and status live in the epic and GitHub Projects. -->
 
| Epic | Capability delivered | Issue |
| --- | --- | --- |
| <Epic name> | <e.g. communication, feedback, control system> | <#issue> |
| <Epic name> | <Capability> | <#issue> |
 
## 5. Scope
 
<!-- Be explicit about what is out. Every deferred item points to a follow-up issue. -->
 
**In scope:**
 
- <Item>
- <Item>
**Out of scope / Deferred:**
 
- <Item> → <Follow-up issue>
- <Coordination with another subsystem, deferred to a later initiative> → <Follow-up issue>
## 6. Deliverables
 
<!-- Concrete things that will exist when the initiative is done, and where each lives. -->
 
- [ ] Implemented software and firmware (repo)
- [ ] <Communication network> and documented message definitions (<repo / Notion>)
- [ ] <Subsystem> control system and configuration (repo)
- [ ] <Subsystem> wiring and setup documentation (Notion)
- [ ] Operating documentation (Notion)
## 7. Interfaces
 
<!-- What this initiative connects to, and what is agreed for each connection. -->
 
| Connection | What is exchanged | Agreement |
| --- | --- | --- |
| Authority controller ↔ <Controller> over <Protocol> | <Approved targets, status, states, faults> | <Units, message definitions, rate, timeout, and authority rules> |
| <Controller> ↔ <Motor controllers / drivers> over <Protocol> | <Individual actuator targets and feedback> | <Node IDs, limits, firmware, and fault handling> |
| <Sensor> → <Controller / Driver> over <Interface> | <Directly measured quantity> | <Mounting location, measured versus inferred quantities, reference, and configuration> |
| <Physical output> → control system over <Interface> | <Actual physical state after mechanical transmission> | <Measurement or estimation method, units, accuracy, update rate, and invalid-data handling> |
 
## 8. Dependencies
 
<!-- What must be ready from other people, teams, or initiatives. Blocking issues are set in GitHub Projects; list one here only to say exactly what is needed from it. -->
 
| Dependency | What we need | Owner |
| --- | --- | --- |
| Mechanical and electrical integration | <Components mounted to the chassis for the demonstration> | <Person or team> |
| Test fixture | <Approved fixture built, if the robot is unavailable> | <Person or team> |
| <Shared resource> — shared with <Initiative #> | <Decision to agree before either initiative completes, e.g. bus, wiring, or protocol> | <Person or team> |
 
## 9. Design Decisions
 
<!-- Decisions to make before the affected work is marked Ready. Record the outcome, or link to where it is recorded in Notion. -->
 
| Decision | What must be defined | Outcome |
| --- | --- | --- |
| Command authority | Responsibilities of each controller, including where faults are handled and reported | <Outcome or link> |
| Physical feedback | Sensor locations, measurement or estimation method, calibration, and reference strategy | <Outcome or link> |
| Operating ranges | Travel and operating limits, target tolerance, completion time, load, and support conditions | <Outcome or link> |
| Fault responses | Response and timing for stale commands, sensor loss, communication loss, or controller faults; which faults allow automatic recovery and which require re-arming | <Outcome or link> |
 
## 10. Risks
 
<!-- Specific risks for this initiative: what could happen, what it would affect, and how it is handled. -->
 
| Risk | Description | Mitigation | Owner |
| --- | --- | --- | --- |
| Interface mismatch | <What could break between connected systems> | <Test the agreed arrangement with all subsystems connected> | <Person> |
| Feedback loss or wrong reference | <What happens if feedback is lost or the reference is wrong> | <How it is detected and the response> | <Person> |
| Uncontrolled motion or load release | <What could move or drop, and what it would affect> | <Mechanical support, fault response, and test precautions> | <Person> |
| Hardware or integration delay | <What is late and what it holds up> | Use the approved test fixture and record what remains to check on the robot | <Person> |
 
**Safe response during testing:** <e.g. e-stop, cut power, support the load>
 
## 11. Documentation
 
<!-- Links only; the detail lives in the linked pages. Fill in evidence and versions once the demonstration is done. -->
 
- **Design / CAD / Schematics:** <Links>
- **Datasheets / Technical documentation:** <Links>
- **Demonstration and test evidence:** <Notion link to results, photos, or video>
- **Tested version:** <Exact Git commit tested>
- **Merged version:** <Link to the annotated Git tag>
- **Documentation SOP:** <Link>
## 12. Definition of Done
 
<!-- The same for every initiative. Only change a line if it truly doesn't apply. -->
 
- [ ] Every success criterion in Section 3 is met, and the demonstration evidence is linked in Section 11.
- [ ] Every epic is complete, or deferred with a follow-up issue.
- [ ] Interfaces are confirmed with the connected systems, using the arrangements agreed with related initiatives.
- [ ] Changes between the tested and merged versions are checked, and affected tests repeated.
- [ ] Documentation follows the SOP and lets a new member set up and operate the deliverable.
- [ ] Remaining limitations are recorded as follow-up issues.
- [ ] The initiative lead and reviewers confirm completion.
## 13. When Done
 
<!-- The same for every initiative. Follow these steps in order once Section 12 is complete. -->
 
1. Create an annotated Git tag on the commit in main that contains the completed initiative, push it to GitHub, and link it in Section 11.
2. Add the tag and a one-line summary of the delivered capability to the Notion document.
3. Close this issue.
