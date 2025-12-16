import { JwtOptions } from "./types.mjs";
import { GenericEndpointContext } from "@better-auth/core";

//#region src/plugins/jwt/sign.d.ts

declare function getJwtToken(ctx: GenericEndpointContext, options?: JwtOptions | undefined): Promise<string>;
//#endregion
export { getJwtToken };
//# sourceMappingURL=sign.d.mts.map