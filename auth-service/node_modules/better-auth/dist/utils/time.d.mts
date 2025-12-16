//#region src/utils/time.d.ts
type Years = "years" | "year" | "yrs" | "yr" | "y";
type Months = "months" | "month" | "mo";
type Weeks = "weeks" | "week" | "w";
type Days = "days" | "day" | "d";
type Hours = "hours" | "hour" | "hrs" | "hr" | "h";
type Minutes = "minutes" | "minute" | "mins" | "min" | "m";
type Seconds = "seconds" | "second" | "secs" | "sec" | "s";
type Unit = Years | Months | Weeks | Days | Hours | Minutes | Seconds;
type UnitAnyCase = Capitalize<Unit> | Uppercase<Unit> | Unit;
type Suffix = " ago" | " from now";
type Prefix = "+" | "-" | "+ " | "- ";
type BaseTimeString = `${number}${UnitAnyCase}` | `${number} ${UnitAnyCase}`;
/**
 * A typed string representing a time duration.
 * Supports formats like "7d", "30m", "1 hour", "2 hours ago", "-5m", etc.
 */
type TimeString = BaseTimeString | `${BaseTimeString}${Suffix}` | `${Prefix}${BaseTimeString}`;
//#endregion
export { TimeString };
//# sourceMappingURL=time.d.mts.map