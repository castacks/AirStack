# Embodiment-neutral goal intake and routing

`rrm/goal_contracts.py` adds a proposal-only layer before the existing embodiment-bound
`TaskRequest` adapters. It prevents the narrow AirStack text grammar or the Kuka-Allegro
fixture from becoming the definition of RRM goal handling.

An operator goal may be qualitative and need not contain coordinates or control
parameters. `GoalRequest` retains the objective, context references, required semantic
operations/resources, qualitative constraints, and an optional embodiment preference.
An embodiment preference constrains routing; it does not change the meaning of the goal.

`route_goal` compares requirements with C03 capability declarations. It returns one
selected embodiment, multiple candidates for later policy/feasibility ranking,
`UNSUPPORTED` when no declared body has the semantics/resources, or `UNAVAILABLE` when
a matching body exists but its required resources are not currently available. It does
not claim target reachability, physical feasibility, permission, safety, or admission.

After routing, an embodiment adapter may produce a `ParameterResolution`. Numeric values
must carry a source and immutable source reference: operator input, observation,
constraint profile, versioned policy, or adapter default. Missing material information
is `NEEDS_CLARIFICATION`; a known physical failure is `INFEASIBLE`. Authorization and
safety rejection deliberately do not appear in that enum because C06 owns those
decisions independently.

The intended progression is:

```text
qualitative goal
  -> semantic/context grounding
  -> capability candidates
  -> embodiment feasibility/policy selection
  -> adapter parameter resolution with provenance
  -> independent permission and safety decision
  -> single-use admission and closed-loop execution
```

`bind_selected_route_to_c01` now provides the first explicit downstream bridge. It
accepts only a route with exactly one selected embodiment, rechecks the referenced C03
capability and current resource availability, requires externally supplied constraint,
issuer, and permission revisions, and produces the existing proposal-only C01
`TaskRequest`. The binding retains the complete immutable qualitative goal so its
constraints are not lost during translation.

The Kuka-Allegro shadow fixture exercises this path through C01–C05 and still returns
`execution_dispatch=false` with semantic-only feasibility references. This is a dry-run
contract result, not physical feasibility, C06 authorization, or hand task completion.
The command GUI now exposes that exact fixture-backed placement goal as a visibly
non-executing preview and persists hash-addressed goal, route, C01, C04, and C05 records.
It is not connected to live hand observations, numeric feasibility, the hand execution
boundary, or simulator actions, and it does not translate arbitrary objectives into
adapter-specific semantics. Existing live execution paths remain unchanged.
