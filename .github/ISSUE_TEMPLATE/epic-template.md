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
- **Contributors:** <Members>
- **Projected Deadline:** <Competition, demonstration, or target date>
- **Notion Document:** <Notion Epic Link>

## Description

<Describe the problem, why it matters to the robot, and the capability this epic will provide. Include the current state and intended outcome.>

## Goals

<What measurable outcomes should this epic achieve?>

- <Goal 1>
- <Goal 2>
- <Goal 3>

## Epic MVP

<Describe the smallest complete capability that would make this epic useful and demonstrable. Specify operating conditions and measurable performance where relevant.>

**MVP demonstration:**  
<“Given [setup], the robot can [behaviour], meeting [criteria].”>

## Scope

**Included:**

- <Required capability or work>
- <Required capability or work>

**Excluded / Deferred:**

- <Capability intentionally left for a later epic>
- <Optional improvement>

## Deliverables

<List the concrete outputs this epic must produce. Include only applicable items.>

- [ ] <Implemented software, assembled hardware, or integrated subsystem>
- [ ] <Interface definitions, configuration, CAD, wiring diagrams, or schematics>
- [ ] <Test results and demonstration evidence>
- [ ] <Setup, operation, and troubleshooting documentation>

## Interfaces and Dependencies

**Interfaces:**  
<Describe connections to other subsystems: ROS topics and message types, CAN messages, power requirements, mechanical mounting, coordinate frames, units, or other relevant contracts.>

| Dependency | Responsible Person / Team | Required By |
|---|---|---|
| <Hardware, upstream task, or subsystem capability> | <Owner> | <Milestone or date> |

## Acceptance Criteria — What Makes This Epic Complete?

<Use observable pass/fail conditions. Replace vague statements such as “works reliably” with defined conditions and thresholds.>

- [ ] <Required behaviour demonstrated under specified conditions>
- [ ] <Performance target met, including units and tolerance>
- [ ] <Relevant failure behaviour verified: stale commands, sensor loss, communications loss, or other faults>
- [ ] <Required interfaces verified with the connected subsystems>
- [ ] <MVP demonstration completed and evidence linked>

## Validation

- **Validation plan:** <Link to section in Notion>
- **Validation evidence:** <Link to section in Notion>
- **Reviewer(s):** <Subsystem reviewer and repo admin>

## Risks and Safety

<List specific risks relevant to this epic, including technical uncertainty, hardware availability, integration, and testing hazards.>

| Risk | Mitigation / Test Precaution | Owner |
|---|---|---|
| <Risk> | <Action to reduce or control it> | <Person> |

## Definition of Done

- [ ] All acceptance criteria are met.
- [ ] Required sub-issues are completed; deferred work is explicitly recorded.
- [ ] Applicable code and design changes are reviewed and merged into the agreed project baseline.
- [ ] Validation evidence is documented.
- [ ] Documentation allows another team member to set up and operate the deliverable.
- [ ] Documentation follows the Documentation SOP and allows a new member to set up and operate the deliverable.
- [ ] Annotated Git tag is pushed and linked in the Notion epic document.
- [ ] Remaining limitations and follow-up issues are documented.
- [ ] Epic lead and relevant subsystem reviewer confirm completion.

## After Merging
1. Create an annotated Git tag on the commit in main that contains the completed epic, and push the tag to GitHub.
2. Update the relevant documentation on Notion with the tag name, a link to the tagged version, and a brief summary of the delivered capability.

## References

- **Design / CAD / Schematics:** <Links>
- **Repository / Pull Requests:** <Links>
- **Documentation (Notion & Local to Repository)  / Datasheets:** <Links>
- **Related Epics:** <Links>
