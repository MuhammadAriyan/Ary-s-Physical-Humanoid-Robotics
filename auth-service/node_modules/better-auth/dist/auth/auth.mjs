import { init } from "../context/init.mjs";
import { createBetterAuth } from "./base.mjs";

//#region src/auth/auth.ts
/**
* Better Auth initializer for full mode (with Kysely)
*
* Check `minimal.ts` for minimal mode (without Kysely)
*/
const betterAuth = (options) => {
	return createBetterAuth(options, init);
};

//#endregion
export { betterAuth };
//# sourceMappingURL=auth.mjs.map