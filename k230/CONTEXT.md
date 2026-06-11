# K230 Visual Targeting

Camera-based targeting context for detecting visual targets and guiding a robot arm toward or away from them.

## Language

**Red target**:
A visually detected red object that can drive robot-arm motion.
_Avoid_: red block, red blob, red thing

**Red pixel area**:
Count of red-classified pixels belonging to detected red target in one frame.
_Avoid_: red size, red area, blob size

**Retreat**:
A commanded increase in separation between robot arm and red target when the target appears too close.
_Avoid_: back off a bit, move away, reverse

## Relationships

- A **Red target** has one **Red pixel area** per frame
- Excessive **Red pixel area** triggers **Retreat**

## Example dialogue

> **Dev:** "When should **Retreat** start?"
> **Domain expert:** "When **Red pixel area** crosses threshold that means **Red target** is too close."

## Flagged ambiguities

- "红色像素面积" was ambiguous between pixel count and bounding-box size — resolved: it means **Red pixel area** as pixel count.
