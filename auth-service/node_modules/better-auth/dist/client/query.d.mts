import { SessionQueryParams } from "./types.mjs";
import { ClientFetchOption } from "@better-auth/core";
import { PreinitializedWritableAtom } from "nanostores";
import { BetterFetch, BetterFetchError } from "@better-fetch/fetch";

//#region src/client/query.d.ts
declare const useAuthQuery: <T>(initializedAtom: PreinitializedWritableAtom<any> | PreinitializedWritableAtom<any>[], path: string, $fetch: BetterFetch, options?: (((value: {
  data: null | T;
  error: null | BetterFetchError;
  isPending: boolean;
}) => ClientFetchOption) | ClientFetchOption) | undefined) => PreinitializedWritableAtom<{
  data: null | T;
  error: null | BetterFetchError;
  isPending: boolean;
  isRefetching: boolean;
  refetch: (queryParams?: {
    query?: SessionQueryParams;
  } | undefined) => Promise<void>;
}> & object;
//#endregion
export { useAuthQuery };
//# sourceMappingURL=query.d.mts.map