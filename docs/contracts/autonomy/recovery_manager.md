# recovery_manager Contract (v1)

## Responsibilities
- Handle blocked/no-progress conditions deterministically.
- Execute ordered strategy chains with bounded retries.

## Inputs
- Recovery trigger events from `navigation_manager`/`progress_monitor`.
- Current mission/nav context.

## Outputs
- Recovery action requests: `HOLD`, `RETRY`, `REPLAN`, `ABORT`.
- Recovery status:
  - attempt count
  - strategy in progress
  - terminal outcome

## Policy
- Configurable `retry_budget` per trigger category.
- Strategy chain default:
  1. `HOLD`
  2. `RETRY`
  3. `REPLAN`
  4. `ABORT`

## Fault Behavior
- If budget exhausted: emit terminal recovery failure with explicit reason and recommendation to `mission_executive` to abort.
