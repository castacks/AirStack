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

This increment does not wire the new intake contract into the live drone console or the
hand boundary. Existing execution paths remain unchanged until an adapter explicitly
accepts a selected route and preserves all current C06/C08/C09 checks.
