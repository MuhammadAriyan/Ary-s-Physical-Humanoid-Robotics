import { CaptchaOptions } from "./types.mjs";
import * as _better_auth_core2 from "@better-auth/core";

//#region src/plugins/captcha/index.d.ts
declare const captcha: (options: CaptchaOptions) => {
  id: "captcha";
  onRequest: (request: Request, ctx: _better_auth_core2.AuthContext) => Promise<{
    response: Response;
  } | undefined>;
};
//#endregion
export { captcha };
//# sourceMappingURL=index.d.mts.map