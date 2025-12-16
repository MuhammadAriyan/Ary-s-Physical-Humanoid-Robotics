import { Jwk, JwtOptions } from "./types.mjs";
import { GenericEndpointContext } from "@better-auth/core";
import * as jose0 from "jose";

//#region src/plugins/jwt/utils.d.ts

declare function generateExportedKeyPair(options?: JwtOptions | undefined): Promise<{
  publicWebKey: jose0.JWK;
  privateWebKey: jose0.JWK;
  alg: "EdDSA" | "ES256" | "ES512" | "PS256" | "RS256";
  cfg: {
    crv?: "Ed25519" | undefined;
  } | {
    crv?: never | undefined;
  } | {
    crv?: never | undefined;
  } | {
    modulusLength?: number | undefined;
  } | {
    modulusLength?: number | undefined;
  };
}>;
/**
 * Creates a Jwk on the database
 *
 * @param ctx
 * @param options
 * @returns
 */
declare function createJwk(ctx: GenericEndpointContext, options?: JwtOptions | undefined): Promise<Jwk>;
//#endregion
export { createJwk, generateExportedKeyPair };
//# sourceMappingURL=utils.d.mts.map