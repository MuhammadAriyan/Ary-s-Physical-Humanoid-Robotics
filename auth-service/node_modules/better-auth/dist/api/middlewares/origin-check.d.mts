import { GenericEndpointContext } from "@better-auth/core";
import * as better_call683 from "better-call";

//#region src/api/middlewares/origin-check.d.ts

/**
 * A middleware to validate callbackURL and origin against
 * trustedOrigins.
 */
declare const originCheckMiddleware: (inputContext: better_call683.MiddlewareInputContext<better_call683.MiddlewareOptions>) => Promise<void>;
declare const originCheck: (getValue: (ctx: GenericEndpointContext) => string | string[]) => (inputContext: better_call683.MiddlewareInputContext<better_call683.MiddlewareOptions>) => Promise<void>;
//#endregion
export { originCheck, originCheckMiddleware };
//# sourceMappingURL=origin-check.d.mts.map