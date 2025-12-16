import * as _better_auth_core20 from "@better-auth/core";
import * as better_call240 from "better-call";

//#region src/integrations/tanstack-start.d.ts
declare const tanstackStartCookies: () => {
  id: "tanstack-start-cookies";
  hooks: {
    after: {
      matcher(ctx: _better_auth_core20.HookEndpointContext): true;
      handler: (inputContext: better_call240.MiddlewareInputContext<better_call240.MiddlewareOptions>) => Promise<void>;
    }[];
  };
};
//#endregion
export { tanstackStartCookies };
//# sourceMappingURL=tanstack-start.d.mts.map