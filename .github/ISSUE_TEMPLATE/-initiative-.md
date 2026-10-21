---
name: "[INITIATIVE]"
about: Use this when creating initiatives.
title: ''
labels: ''
assignees: maisonJar-dev
type: INITIATIVE

---

# [INITIATIVE] <Initiative Name>

## Initiative Details

- **Initiative Lead:** <Person responsible for coordination and completion>
- **Projected Deadline:** <Month Day, Year>
- **Notion Document:** <Notion Initiative Link>

## Description

<Describe the subsystem on EMBR, what currently works at the time of creating this initiative, the problem being addressed, and the capability the robot will have once this initiative is complete.>

## Goals

- **<Capability>:** <Observable outcome on the robot>
- **<Capability>:** <Observable outcome on the robot>
- **Operating Constraints — <Constraint Name>:** <Allowed modes, sequencing rules (e.g. one motion mode at a time), limits, and operating conditions. Note any stretch goals, such as simultaneous operation, if plausible.>
- **Physical Feedback:** <What final physical state is known, and whether it is measured or estimated. State the accuracy and reference strategy, or mark as TBD.>

## Epics

<Break the initiative into epics. Each epic should deliver an independently testable capability. Create each epic from the EPIC template and link it here.>

| Epic | Capability Delivered | Epic Lead | Issue / Status |
| --- | --- | --- | --- |
| <Epic name> | <e.g. communication, feedback, control system, validation> | <Person> | <#Issue — status> |
| <Epic name> | <Capability> | <Person> | <#Issue — status> |

## Initiative MVP

**Required validation:** <State which stages are required and how many physical options must be completed, e.g. "Complete both virtual/controller logic validation and physics simulation, plus at least one physical option below.">

- **[REQUIRED / DEFERRED / N/A] Virtual / Controller Logic:** <In RViz2 or equivalent, describe what can be commanded on a basic version of EMBR. Verify controller logic, limits, and responses to injected faults through recorded results, using visualization to inspect behaviour.>
- **[REQUIRED / DEFERRED / N/A] Physics Simulation:** <In our simulation software, test the system as closely as possible to the physical setup. Validate actuator effort, loading, and failsafes. Document simulation limitations.>
- **[PHYSICAL OPTION 1] Robot:** <Conditions under which testing happens on the assembled robot, e.g. which mechanical and electrical components must be mounted.>
- **[PHYSICAL OPTION 2] Approved Test Fixture:** <Fallback setup if the robot is unavailable. Document its representative load, geometry, what it proves, and validation limitations.> <Fallback approver: Person>

**Required for Completion:**

<Summarize what must be true for the MVP to be complete: required validation stages, documented failsafe behaviour, startup and arming requirements, and the coordinated behaviour the system must achieve within defined limits and tolerances.>

**MVP demonstration:**

Given that <hardware is set up, communication is established with all controllers/devices using <protocol>, and feedback loops are set up (see documentation)>, the robot or approved test fixture can <behaviour>.

<Describe operating constraints that apply during the demonstration.> Demonstrate startup, controlled operation, shutdown, and the documented fault responses. Record the results and any limitations of the test setup.

## Scope

**Included:**

- <Communication protocol>
- <Feedback interface>
- <Physical state feedback>
- <Control system and failsafes>
- <Kinematics, planning, or other supporting logic>

**Excluded / Deferred:**

- <Excluded capability or interface>
- <Coordination with other subsystems deferred to a later initiative, and related follow-up issue>

## Deliverables

- [ ] Implemented software and firmware where applicable
- [ ] <Communication network> and documented message definitions
- [ ] <Subsystem>-specific control system and configuration
- [ ] <Subsystem> wiring and setup documentation
- [ ] Validation results and operating documentation

## Interfaces and Dependencies

**Interfaces:**

| Connection | Purpose | Contract / Configuration |
| --- | --- | --- |
| Authority controller ↔ <Controller> over <Protocol> | <Approved targets, status, states, faults> | <Units, message definitions, rate, timeout, and authority rules: TBD> |
| <Controller> ↔ <Motor controllers / drivers> over <Protocol> | <Individual actuator targets and feedback> | <Node IDs, limits, firmware, and fault handling: TBD> |
| <Sensor> → <Controller / Driver> over <Interface> | <Directly measured quantity> | <Sensor mounting location, directly measured versus inferred quantities, reference, and configuration: TBD> |
| <Physical output> → control system, interface <TBD> | <Actual physical state after mechanical transmission> | <Measurement or estimation method, units, reference, accuracy, update rate, and invalid-data handling: TBD> |

**Dependencies:**

| Dependency | Required Capability / Readiness | Owner | Blocking Issue / Status |
| --- | --- | --- | --- |
| Mechanical and electrical integration | <Components mounted to the chassis for Physical Option 1> | <Person or team> | <Link or status> |
| Physical test fixture | <Approved fallback construction available if Physical Option 2 is used> | <Person or team> | <Link or status> |
| <Controller> communication | <Communication with all controllers and devices using <protocol>> | <Person or team> | <Link or status> |
| Feedback | <Input and output feedback loops set up; final physical state known> | <Person or team> | <Link or status> |
| <Shared resource> — shared with <Initiative #> | <Shared decision required, e.g. bus, wiring, or protocol configuration. Agree before completing either initiative.> | <Person or team> | <#Issue — shared decision TBD> |

**Design decisions to resolve before affected implementation is Ready:**

- **Control ownership and command authority:** <Define the responsibilities of each controller, including where faults are handled and reported.>
- **Physical feedback:** <Define sensor locations, measurement or estimation methods, calibration, and reference strategy.>
- **Operating ranges and performance:** <Define travel and operating limits, target tolerance, completion time, load, and support conditions.>
- **Fault responses:** <Define the coordinated response and response time for stale commands, sensor loss, communications loss, or controller faults. Specify which faults permit automatic recovery and which require explicit re-arming.>
- **Physical fallback approval:** <Confirm who approves the physical fallback, its validation limitations, and any checks deferred until the robot is available.>

## Acceptance Criteria — What Makes This Initiative Complete?

<Use observable pass/fail conditions. Resolve numerical placeholders before affected implementation is marked Ready. Mark inapplicable criteria with a reason. Change criteria as needed for each initiative.>

- [ ] Startup is repeatable, verifies valid calibration, feedback, and a known reference, and prevents actuation until explicitly armed.
- [ ] All <number> actuators/axes are individually addressable and mapped to the correct physical component.
- [ ] <Operating and travel> limits are documented and enforced.
- [ ] <Operating constraint, e.g. only one motion mode is active at a time>; conflicting requests are rejected or queued according to a documented rule.
- [ ] Targets are reached within <tolerance and units>, within <time>, under <load and support conditions>.
- [ ] Feedback accuracy is verified against <independent measurement or reference> within <tolerance>.
- [ ] Loss of commands, feedback, or one controller triggers the documented coordinated response within <time>.
- [ ] Startup, shutdown, and fault recovery do not cause unintended motion.
- [ ] Required interfaces are verified with the connected systems, using the arrangement agreed with <related initiative #>.
- [ ] Required virtual, physics, and physical validation is completed and demonstrates the documented fault behaviour.
- [ ] Physical testing demonstrates that the chosen fault response prevents <uncontrolled motion, descent, or load release>.
- [ ] All required epics are complete; deferred epics are recorded with follow-up issues.
- [ ] The MVP demonstration is completed and evidence is documented.
- [ ] Documentation according to the SOP is complete.

## Validation

- **Validation plan:** <Link to section in Notion>
- **Validation evidence:** <Link to section in Notion>
- **Reviewer(s):** <Subsystem reviewer and repo admin>
- **Tested version:** <Exact Git commit tested>
- **Merged version:** <Link to the annotated Git tag created after merging>

## Risks and Safety

<List specific technical uncertainties, integration risks, hardware dependencies, and testing hazards. Define the appropriate safe response for this subsystem.>

| Risk | Mitigation / Test Precaution | Owner |
| --- | --- | --- |
| <Protocol or interface compatibility> | <Validate the arrangement agreed with related initiatives, including communication and fault handling with all subsystems connected.> | <Person> |
| <Feedback loss or incorrect reference> | <Define detection and the coordinated response for sensor loss or an invalid reference: TBD> | <Person> |
| <Uncontrolled motion or load release> | <Define mechanical support, fault responses, and physical test precautions used to verify it is prevented: TBD> | <Person> |
| Hardware or integration delay | Use the approved Physical Option 2 if the robot is unavailable. Record validation limitations and any remaining robot-specific checks. | <Person> |

## Definition of Done

- [ ] All acceptance criteria are met.
- [ ] Required epics and sub-issues are completed; deferred work is explicitly recorded.
- [ ] Applicable code and design changes are reviewed and merged into the agreed project baseline.
- [ ] Validation evidence is documented and identifies the exact commit tested.
- [ ] Changes between the tested and merged versions are assessed, with affected checks repeated where necessary.
- [ ] Documentation follows the Documentation SOP and allows a new member to set up and operate the deliverable.
- [ ] An annotated Git tag is pushed and linked in the Notion initiative document.
- [ ] Remaining limitations and follow-up issues are documented.
- [ ] Initiative lead, relevant subsystem reviewer, and repo admin confirm completion.

## After Merging

1. Create an annotated Git tag on the commit in main that contains the completed initiative, and push the tag to GitHub.
2. Update the Notion initiative with the tag name, a link to the tagged version, and a brief summary of the delivered capability.
3. Preserve the exact tested commit separately from the merged-version tag, and link any integration verification results.

## References

- **Design / CAD / Schematics:** <Links>
- **Repository / Pull Requests:** <Links>
- **Documentation SOP:** <Link>
- **Documentation (Notion and Repository) / Datasheets:** <Links>
- **Epics:** <Links>
- **Related Initiatives:** <Initiative # — shared dependency>
