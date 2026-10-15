---
name: EPIC Template
about: Use this when creating epics.
title: "[EPIC]"
labels: ''
assignees: maisonJar-dev
type: EPIC

---

# [EPIC] <Epic Name>

## Epic Details

- **Epic Lead:** <Person responsible for coordination and completion>
- **Projected Deadline:** <Month Day, Year>
- **Notion Document:** <Notion Epic Link>

## Description

<Describe the subsystem, its current capabilities, the problem being addressed, and the capability this epic will deliver.>

## Goals

- **<Capability>:** <Observable outcome>
- **<Capability>:** <Observable outcome>
- **Operating Constraints:** <Allowed modes, sequencing rules, limits, and operating conditions>
- **Feedback:** <What is measured or estimated and how a known reference is established>

## Epic MVP

<Describe the smallest complete capability that makes this epic useful and demonstrable. Mark each validation stage as required, deferred, or not applicable.>

- **Virtual / Controller Logic:** <Verify commands, state transitions, limits, and injected faults through recorded results. Use visualization to inspect behaviour.>
- **Physics Simulation:** <Validate relevant dynamics, loading, actuator effort, and fault responses. Document simulation limitations.>
- **Physical — Robot:** <Demonstrate the required capability on the assembled robot under defined conditions.>
- **Physical — Approved Test Fixture:** <Describe the fallback setup, representative load and geometry, what it proves, and what remains unvalidated.>
- **Required for Completion:** <Identify mandatory validation stages, accepted alternatives, and who approves a fallback.>

**MVP demonstration:**

Given <setup, initial state, load, and operating conditions>, the system can <behaviour>, meeting <measurable criteria>, while enforcing <limits and fault responses>.

## Scope

**Included:**

- <Capability or control function>
- <Communication interface or protocol>
- <Feedback and reference strategy>
- <Required integration and validation>

**Excluded / Deferred:**

- <Excluded capability or interface>
- <Deferred improvement and related follow-up issue>

## Deliverables

- [ ] <Implemented software or firmware>
- [ ] <Communication network and documented message definitions>
- [ ] <Subsystem control logic and configuration>
- [ ] <Hardware wiring, setup, or test fixture>
- [ ] <Validation results and operating documentation>

## Interfaces and Dependencies

**Interfaces:**

| Connection | Purpose | Contract / Configuration |
| --- | --- | --- |
| <Command source> ↔ <Controller> over <Protocol> | <Targets, mode, status, and faults> | <Units, message definitions, rate, timeout, and authority rules> |
| <Controller> ↔ <Actuator / Driver> over <Protocol> | <Commands and feedback> | <Device IDs, limits, firmware, and fault handling> |
| <Sensor> → <Controller / Driver> over <Interface> | <Directly measured quantity> | <Units, reference, accuracy, update rate, and invalid-data handling> |

**Dependencies:**

| Dependency | Required Capability / Readiness | Owner | Blocking Issue / Status |
| --- | --- | --- | --- |
| <Mechanical / Electrical / Software dependency> | <What must be available> | <Person or team> | <Link or status> |

**Design decisions to resolve before affected implementation is Ready:**

- <Control ownership and command authority>
- <Feedback location, calibration, homing, and reference strategy>
- <Operating ranges, performance thresholds, and fault responses>
- <Hardware, protocol, and firmware compatibility>

## Acceptance Criteria — What Makes This Epic Complete?

<Use observable pass/fail conditions. Resolve numerical placeholders before affected implementation is marked Ready. Mark inapplicable criteria with a reason.Change criteria as needed for each epic>

- [ ] Startup verifies required calibration, feedback, and references; actuation remains disabled until explicitly armed.
- [ ] Each device or actuator is individually addressable and mapped to the correct physical component.
- [ ] Required operating modes and transitions behave as documented.
- [ ] Conflicting commands are handled according to a documented rule.
- [ ] Operating and actuator limits are documented and enforced.
- [ ] Targets are reached within <tolerance and units>, within <time>, under <load and operating conditions>.
- [ ] Feedback accuracy is verified against <independent measurement or reference> within <tolerance>.
- [ ] Loss of commands, feedback, or a connected device triggers <defined response> within <time>.
- [ ] Startup, shutdown, and fault recovery do not cause unintended motion.
- [ ] Required interfaces are verified with connected systems.
- [ ] Required simulation and/or physical validation demonstrates the documented fault behaviour. 
- [ ] The MVP required for completion is completed and is documented.
- [ ] The MVP demonstration is completed and evidence is documented.
- [ ] Documentation according to the SOP is complete.

## Validation

- **Validation plan:** <Link to section in Notion>
- **Validation evidence:** <Link to section in Notion>
- **Reviewer(s):** <Subsystem reviewer and repo admin>
- **Tested version:** <Exact Git commit tested>
- **Merged version:** <Link to annotated Git tag created after merging>

## Risks and Safety

<List specific technical uncertainties, integration risks, hardware dependencies, and testing hazards. Define the appropriate safe response for this subsystem.>

| Risk | Mitigation / Test Precaution | Owner |
| --- | --- | --- |
| <Protocol or interface incompatibility> | <Verify configuration and identifiers; document fallback> | <Person> |
| <Feedback loss or incorrect reference> | <Detection, validation, and defined response> | <Person> |
| <Uncontrolled motion or load release> | <Mechanical support, limits, and verified fault response> | <Person> |
| <Hardware or integration delay> | <Approved fallback and validation limitations> | <Person> |

## Definition of Done

- [ ] All acceptance criteria are met.
- [ ] Required sub-issues are completed; deferred work is explicitly recorded.
- [ ] Applicable code and design changes are reviewed and merged into the agreed project baseline.
- [ ] Validation evidence is documented and identifies the exact commit tested.
- [ ] Changes between the tested and merged versions are assessed, with affected checks repeated where necessary.
- [ ] Documentation follows the Documentation SOP and allows a new member to set up and operate the deliverable.
- [ ] An annotated Git tag is pushed and linked in the Notion epic document.
- [ ] Remaining limitations and follow-up issues are documented.
- [ ] Epic lead, relevant subsystem reviewer, and repo admin confirm completion.

## After Merging

1. Create an annotated Git tag on the commit in main that contains the completed epic, and push the tag to GitHub.
2. Update the Notion epic with the tag name, a link to the tagged version, and a brief summary of the delivered capability.
3. Preserve the exact tested commit separately from the merged-version tag, and link any integration verification results.

## References

- **Design / CAD / Schematics:** <Links>
- **Repository / Pull Requests:** <Links>
- **Documentation SOP:** <Link>
- **Documentation (Notion and Repository) / Datasheets:** <Links>
- **Related Epics:** <Links>
