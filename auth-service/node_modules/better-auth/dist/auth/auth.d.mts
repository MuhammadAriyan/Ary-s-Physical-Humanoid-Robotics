import { Auth } from "../types/auth.mjs";
import "../types/index.mjs";
import { BetterAuthOptions } from "@better-auth/core";

//#region src/auth/auth.d.ts

/**
 * Better Auth initializer for full mode (with Kysely)
 *
 * Check `minimal.ts` for minimal mode (without Kysely)
 */
declare const betterAuth: <Options extends BetterAuthOptions>(options: Options & Record<never, never>) => Auth<Options>;
//#endregion
export { betterAuth };
//# sourceMappingURL=auth.d.mts.map