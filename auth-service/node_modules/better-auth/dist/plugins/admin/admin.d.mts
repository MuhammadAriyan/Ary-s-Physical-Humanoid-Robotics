import { AccessControl, Statements } from "../access/types.mjs";
import { AdminOptions, InferAdminRolesFromOption, SessionWithImpersonatedBy, UserWithRole } from "./types.mjs";
import "../index.mjs";
import * as _better_auth_core29 from "@better-auth/core";
import * as _better_auth_core_db16 from "@better-auth/core/db";
import * as zod1906 from "zod";
import * as better_call638 from "better-call";
import * as zod_v4_core260 from "zod/v4/core";

//#region src/plugins/admin/admin.d.ts
declare const admin: <O extends AdminOptions>(options?: O | undefined) => {
  id: "admin";
  init(): {
    options: {
      databaseHooks: {
        user: {
          create: {
            before(user: {
              id: string;
              createdAt: Date;
              updatedAt: Date;
              email: string;
              emailVerified: boolean;
              name: string;
              image?: string | null | undefined;
            } & Record<string, unknown>): Promise<{
              data: {
                id: string;
                createdAt: Date;
                updatedAt: Date;
                email: string;
                emailVerified: boolean;
                name: string;
                image?: string | null | undefined;
                role: string;
              };
            }>;
          };
        };
        session: {
          create: {
            before(session: {
              id: string;
              createdAt: Date;
              updatedAt: Date;
              userId: string;
              expiresAt: Date;
              token: string;
              ipAddress?: string | null | undefined;
              userAgent?: string | null | undefined;
            } & Record<string, unknown>, ctx: _better_auth_core29.GenericEndpointContext | null): Promise<void>;
          };
        };
      };
    };
  };
  hooks: {
    after: {
      matcher(context: _better_auth_core29.HookEndpointContext): boolean;
      handler: (inputContext: better_call638.MiddlewareInputContext<better_call638.MiddlewareOptions>) => Promise<SessionWithImpersonatedBy[] | undefined>;
    }[];
  };
  endpoints: {
    setRole: better_call638.StrictEndpoint<"/admin/set-role", {
      method: "POST";
      body: zod1906.ZodObject<{
        userId: zod1906.ZodCoercedString<unknown>;
        role: zod1906.ZodUnion<readonly [zod1906.ZodString, zod1906.ZodArray<zod1906.ZodString>]>;
      }, zod_v4_core260.$strip>;
      requireHeaders: true;
      use: ((inputContext: better_call638.MiddlewareInputContext<better_call638.MiddlewareOptions>) => Promise<{
        session: {
          user: UserWithRole;
          session: _better_auth_core_db16.Session;
        };
      }>)[];
      metadata: {
        openapi: {
          operationId: string;
          summary: string;
          description: string;
          responses: {
            200: {
              description: string;
              content: {
                "application/json": {
                  schema: {
                    type: "object";
                    properties: {
                      user: {
                        $ref: string;
                      };
                    };
                  };
                };
              };
            };
          };
        };
        $Infer: {
          body: {
            userId: string;
            role: "user" | "admin" | ("user" | "admin")[];
          };
        };
      };
    }, {
      user: UserWithRole;
    }>;
    getUser: better_call638.StrictEndpoint<"/admin/get-user", {
      method: "GET";
      query: zod1906.ZodObject<{
        id: zod1906.ZodString;
      }, zod_v4_core260.$strip>;
      use: ((inputContext: better_call638.MiddlewareInputContext<better_call638.MiddlewareOptions>) => Promise<{
        session: {
          user: UserWithRole;
          session: _better_auth_core_db16.Session;
        };
      }>)[];
      metadata: {
        openapi: {
          operationId: string;
          summary: string;
          description: string;
          responses: {
            200: {
              description: string;
              content: {
                "application/json": {
                  schema: {
                    type: "object";
                    properties: {
                      user: {
                        $ref: string;
                      };
                    };
                  };
                };
              };
            };
          };
        };
      };
    }, {
      id: string;
      createdAt: Date;
      updatedAt: Date;
      email: string;
      emailVerified: boolean;
      name: string;
      image?: string | null | undefined;
    }>;
    createUser: better_call638.StrictEndpoint<"/admin/create-user", {
      method: "POST";
      body: zod1906.ZodObject<{
        email: zod1906.ZodString;
        password: zod1906.ZodString;
        name: zod1906.ZodString;
        role: zod1906.ZodOptional<zod1906.ZodUnion<readonly [zod1906.ZodString, zod1906.ZodArray<zod1906.ZodString>]>>;
        data: zod1906.ZodOptional<zod1906.ZodRecord<zod1906.ZodString, zod1906.ZodAny>>;
      }, zod_v4_core260.$strip>;
      metadata: {
        openapi: {
          operationId: string;
          summary: string;
          description: string;
          responses: {
            200: {
              description: string;
              content: {
                "application/json": {
                  schema: {
                    type: "object";
                    properties: {
                      user: {
                        $ref: string;
                      };
                    };
                  };
                };
              };
            };
          };
        };
        $Infer: {
          body: {
            email: string;
            password: string;
            name: string;
            role?: "user" | "admin" | ("user" | "admin")[] | undefined;
            data?: Record<string, any> | undefined;
          };
        };
      };
    }, {
      user: UserWithRole;
    }>;
    adminUpdateUser: better_call638.StrictEndpoint<"/admin/update-user", {
      method: "POST";
      body: zod1906.ZodObject<{
        userId: zod1906.ZodCoercedString<unknown>;
        data: zod1906.ZodRecord<zod1906.ZodAny, zod1906.ZodAny>;
      }, zod_v4_core260.$strip>;
      use: ((inputContext: better_call638.MiddlewareInputContext<better_call638.MiddlewareOptions>) => Promise<{
        session: {
          user: UserWithRole;
          session: _better_auth_core_db16.Session;
        };
      }>)[];
      metadata: {
        openapi: {
          operationId: string;
          summary: string;
          description: string;
          responses: {
            200: {
              description: string;
              content: {
                "application/json": {
                  schema: {
                    type: "object";
                    properties: {
                      user: {
                        $ref: string;
                      };
                    };
                  };
                };
              };
            };
          };
        };
      };
    }, UserWithRole>;
    listUsers: better_call638.StrictEndpoint<"/admin/list-users", {
      method: "GET";
      use: ((inputContext: better_call638.MiddlewareInputContext<better_call638.MiddlewareOptions>) => Promise<{
        session: {
          user: UserWithRole;
          session: _better_auth_core_db16.Session;
        };
      }>)[];
      query: zod1906.ZodObject<{
        searchValue: zod1906.ZodOptional<zod1906.ZodString>;
        searchField: zod1906.ZodOptional<zod1906.ZodEnum<{
          name: "name";
          email: "email";
        }>>;
        searchOperator: zod1906.ZodOptional<zod1906.ZodEnum<{
          contains: "contains";
          starts_with: "starts_with";
          ends_with: "ends_with";
        }>>;
        limit: zod1906.ZodOptional<zod1906.ZodUnion<[zod1906.ZodString, zod1906.ZodNumber]>>;
        offset: zod1906.ZodOptional<zod1906.ZodUnion<[zod1906.ZodString, zod1906.ZodNumber]>>;
        sortBy: zod1906.ZodOptional<zod1906.ZodString>;
        sortDirection: zod1906.ZodOptional<zod1906.ZodEnum<{
          asc: "asc";
          desc: "desc";
        }>>;
        filterField: zod1906.ZodOptional<zod1906.ZodString>;
        filterValue: zod1906.ZodOptional<zod1906.ZodUnion<[zod1906.ZodUnion<[zod1906.ZodString, zod1906.ZodNumber]>, zod1906.ZodBoolean]>>;
        filterOperator: zod1906.ZodOptional<zod1906.ZodEnum<{
          eq: "eq";
          ne: "ne";
          lt: "lt";
          lte: "lte";
          gt: "gt";
          gte: "gte";
          contains: "contains";
        }>>;
      }, zod_v4_core260.$strip>;
      metadata: {
        openapi: {
          operationId: string;
          summary: string;
          description: string;
          responses: {
            200: {
              description: string;
              content: {
                "application/json": {
                  schema: {
                    type: "object";
                    properties: {
                      users: {
                        type: string;
                        items: {
                          $ref: string;
                        };
                      };
                      total: {
                        type: string;
                      };
                      limit: {
                        type: string;
                      };
                      offset: {
                        type: string;
                      };
                    };
                    required: string[];
                  };
                };
              };
            };
          };
        };
      };
    }, {
      users: UserWithRole[];
      total: number;
      limit: number | undefined;
      offset: number | undefined;
    } | {
      users: never[];
      total: number;
    }>;
    listUserSessions: better_call638.StrictEndpoint<"/admin/list-user-sessions", {
      method: "POST";
      use: ((inputContext: better_call638.MiddlewareInputContext<better_call638.MiddlewareOptions>) => Promise<{
        session: {
          user: UserWithRole;
          session: _better_auth_core_db16.Session;
        };
      }>)[];
      body: zod1906.ZodObject<{
        userId: zod1906.ZodCoercedString<unknown>;
      }, zod_v4_core260.$strip>;
      metadata: {
        openapi: {
          operationId: string;
          summary: string;
          description: string;
          responses: {
            200: {
              description: string;
              content: {
                "application/json": {
                  schema: {
                    type: "object";
                    properties: {
                      sessions: {
                        type: string;
                        items: {
                          $ref: string;
                        };
                      };
                    };
                  };
                };
              };
            };
          };
        };
      };
    }, {
      sessions: SessionWithImpersonatedBy[];
    }>;
    unbanUser: better_call638.StrictEndpoint<"/admin/unban-user", {
      method: "POST";
      body: zod1906.ZodObject<{
        userId: zod1906.ZodCoercedString<unknown>;
      }, zod_v4_core260.$strip>;
      use: ((inputContext: better_call638.MiddlewareInputContext<better_call638.MiddlewareOptions>) => Promise<{
        session: {
          user: UserWithRole;
          session: _better_auth_core_db16.Session;
        };
      }>)[];
      metadata: {
        openapi: {
          operationId: string;
          summary: string;
          description: string;
          responses: {
            200: {
              description: string;
              content: {
                "application/json": {
                  schema: {
                    type: "object";
                    properties: {
                      user: {
                        $ref: string;
                      };
                    };
                  };
                };
              };
            };
          };
        };
      };
    }, {
      user: {
        id: string;
        createdAt: Date;
        updatedAt: Date;
        email: string;
        emailVerified: boolean;
        name: string;
        image?: string | null | undefined;
      } & Record<string, any>;
    }>;
    banUser: better_call638.StrictEndpoint<"/admin/ban-user", {
      method: "POST";
      body: zod1906.ZodObject<{
        userId: zod1906.ZodCoercedString<unknown>;
        banReason: zod1906.ZodOptional<zod1906.ZodString>;
        banExpiresIn: zod1906.ZodOptional<zod1906.ZodNumber>;
      }, zod_v4_core260.$strip>;
      use: ((inputContext: better_call638.MiddlewareInputContext<better_call638.MiddlewareOptions>) => Promise<{
        session: {
          user: UserWithRole;
          session: _better_auth_core_db16.Session;
        };
      }>)[];
      metadata: {
        openapi: {
          operationId: string;
          summary: string;
          description: string;
          responses: {
            200: {
              description: string;
              content: {
                "application/json": {
                  schema: {
                    type: "object";
                    properties: {
                      user: {
                        $ref: string;
                      };
                    };
                  };
                };
              };
            };
          };
        };
      };
    }, {
      user: {
        id: string;
        createdAt: Date;
        updatedAt: Date;
        email: string;
        emailVerified: boolean;
        name: string;
        image?: string | null | undefined;
      } & Record<string, any>;
    }>;
    impersonateUser: better_call638.StrictEndpoint<"/admin/impersonate-user", {
      method: "POST";
      body: zod1906.ZodObject<{
        userId: zod1906.ZodCoercedString<unknown>;
      }, zod_v4_core260.$strip>;
      use: ((inputContext: better_call638.MiddlewareInputContext<better_call638.MiddlewareOptions>) => Promise<{
        session: {
          user: UserWithRole;
          session: _better_auth_core_db16.Session;
        };
      }>)[];
      metadata: {
        openapi: {
          operationId: string;
          summary: string;
          description: string;
          responses: {
            200: {
              description: string;
              content: {
                "application/json": {
                  schema: {
                    type: "object";
                    properties: {
                      session: {
                        $ref: string;
                      };
                      user: {
                        $ref: string;
                      };
                    };
                  };
                };
              };
            };
          };
        };
      };
    }, {
      session: {
        id: string;
        createdAt: Date;
        updatedAt: Date;
        userId: string;
        expiresAt: Date;
        token: string;
        ipAddress?: string | null | undefined;
        userAgent?: string | null | undefined;
      };
      user: UserWithRole;
    }>;
    stopImpersonating: better_call638.StrictEndpoint<"/admin/stop-impersonating", {
      method: "POST";
      requireHeaders: true;
    }, {
      session: _better_auth_core_db16.Session & Record<string, any>;
      user: _better_auth_core_db16.User & Record<string, any>;
    }>;
    revokeUserSession: better_call638.StrictEndpoint<"/admin/revoke-user-session", {
      method: "POST";
      body: zod1906.ZodObject<{
        sessionToken: zod1906.ZodString;
      }, zod_v4_core260.$strip>;
      use: ((inputContext: better_call638.MiddlewareInputContext<better_call638.MiddlewareOptions>) => Promise<{
        session: {
          user: UserWithRole;
          session: _better_auth_core_db16.Session;
        };
      }>)[];
      metadata: {
        openapi: {
          operationId: string;
          summary: string;
          description: string;
          responses: {
            200: {
              description: string;
              content: {
                "application/json": {
                  schema: {
                    type: "object";
                    properties: {
                      success: {
                        type: string;
                      };
                    };
                  };
                };
              };
            };
          };
        };
      };
    }, {
      success: boolean;
    }>;
    revokeUserSessions: better_call638.StrictEndpoint<"/admin/revoke-user-sessions", {
      method: "POST";
      body: zod1906.ZodObject<{
        userId: zod1906.ZodCoercedString<unknown>;
      }, zod_v4_core260.$strip>;
      use: ((inputContext: better_call638.MiddlewareInputContext<better_call638.MiddlewareOptions>) => Promise<{
        session: {
          user: UserWithRole;
          session: _better_auth_core_db16.Session;
        };
      }>)[];
      metadata: {
        openapi: {
          operationId: string;
          summary: string;
          description: string;
          responses: {
            200: {
              description: string;
              content: {
                "application/json": {
                  schema: {
                    type: "object";
                    properties: {
                      success: {
                        type: string;
                      };
                    };
                  };
                };
              };
            };
          };
        };
      };
    }, {
      success: boolean;
    }>;
    removeUser: better_call638.StrictEndpoint<"/admin/remove-user", {
      method: "POST";
      body: zod1906.ZodObject<{
        userId: zod1906.ZodCoercedString<unknown>;
      }, zod_v4_core260.$strip>;
      use: ((inputContext: better_call638.MiddlewareInputContext<better_call638.MiddlewareOptions>) => Promise<{
        session: {
          user: UserWithRole;
          session: _better_auth_core_db16.Session;
        };
      }>)[];
      metadata: {
        openapi: {
          operationId: string;
          summary: string;
          description: string;
          responses: {
            200: {
              description: string;
              content: {
                "application/json": {
                  schema: {
                    type: "object";
                    properties: {
                      success: {
                        type: string;
                      };
                    };
                  };
                };
              };
            };
          };
        };
      };
    }, {
      success: boolean;
    }>;
    setUserPassword: better_call638.StrictEndpoint<"/admin/set-user-password", {
      method: "POST";
      body: zod1906.ZodObject<{
        newPassword: zod1906.ZodString;
        userId: zod1906.ZodCoercedString<unknown>;
      }, zod_v4_core260.$strip>;
      use: ((inputContext: better_call638.MiddlewareInputContext<better_call638.MiddlewareOptions>) => Promise<{
        session: {
          user: UserWithRole;
          session: _better_auth_core_db16.Session;
        };
      }>)[];
      metadata: {
        openapi: {
          operationId: string;
          summary: string;
          description: string;
          responses: {
            200: {
              description: string;
              content: {
                "application/json": {
                  schema: {
                    type: "object";
                    properties: {
                      status: {
                        type: string;
                      };
                    };
                  };
                };
              };
            };
          };
        };
      };
    }, {
      status: boolean;
    }>;
    userHasPermission: better_call638.StrictEndpoint<"/admin/has-permission", {
      method: "POST";
      body: zod1906.ZodIntersection<zod1906.ZodObject<{
        userId: zod1906.ZodOptional<zod1906.ZodCoercedString<unknown>>;
        role: zod1906.ZodOptional<zod1906.ZodString>;
      }, zod_v4_core260.$strip>, zod1906.ZodUnion<readonly [zod1906.ZodObject<{
        permission: zod1906.ZodRecord<zod1906.ZodString, zod1906.ZodArray<zod1906.ZodString>>;
        permissions: zod1906.ZodUndefined;
      }, zod_v4_core260.$strip>, zod1906.ZodObject<{
        permission: zod1906.ZodUndefined;
        permissions: zod1906.ZodRecord<zod1906.ZodString, zod1906.ZodArray<zod1906.ZodString>>;
      }, zod_v4_core260.$strip>]>>;
      metadata: {
        openapi: {
          description: string;
          requestBody: {
            content: {
              "application/json": {
                schema: {
                  type: "object";
                  properties: {
                    permission: {
                      type: string;
                      description: string;
                      deprecated: boolean;
                    };
                    permissions: {
                      type: string;
                      description: string;
                    };
                  };
                  required: string[];
                };
              };
            };
          };
          responses: {
            "200": {
              description: string;
              content: {
                "application/json": {
                  schema: {
                    type: "object";
                    properties: {
                      error: {
                        type: string;
                      };
                      success: {
                        type: string;
                      };
                    };
                    required: string[];
                  };
                };
              };
            };
          };
        };
        $Infer: {
          body: ({
            permission: { [key in keyof (O["ac"] extends AccessControl<infer S extends Statements> ? S : {
              readonly user: readonly ["create", "list", "set-role", "ban", "impersonate", "delete", "set-password", "get", "update"];
              readonly session: readonly ["list", "revoke", "delete"];
            })]?: ((O["ac"] extends AccessControl<infer S extends Statements> ? S : {
              readonly user: readonly ["create", "list", "set-role", "ban", "impersonate", "delete", "set-password", "get", "update"];
              readonly session: readonly ["list", "revoke", "delete"];
            })[key] extends readonly unknown[] ? (O["ac"] extends AccessControl<infer S extends Statements> ? S : {
              readonly user: readonly ["create", "list", "set-role", "ban", "impersonate", "delete", "set-password", "get", "update"];
              readonly session: readonly ["list", "revoke", "delete"];
            })[key][number] : never)[] | undefined };
            permissions?: never | undefined;
          } | {
            permissions: { [key in keyof (O["ac"] extends AccessControl<infer S extends Statements> ? S : {
              readonly user: readonly ["create", "list", "set-role", "ban", "impersonate", "delete", "set-password", "get", "update"];
              readonly session: readonly ["list", "revoke", "delete"];
            })]?: ((O["ac"] extends AccessControl<infer S extends Statements> ? S : {
              readonly user: readonly ["create", "list", "set-role", "ban", "impersonate", "delete", "set-password", "get", "update"];
              readonly session: readonly ["list", "revoke", "delete"];
            })[key] extends readonly unknown[] ? (O["ac"] extends AccessControl<infer S extends Statements> ? S : {
              readonly user: readonly ["create", "list", "set-role", "ban", "impersonate", "delete", "set-password", "get", "update"];
              readonly session: readonly ["list", "revoke", "delete"];
            })[key][number] : never)[] | undefined };
            permission?: never | undefined;
          }) & {
            userId?: string | undefined;
            role?: InferAdminRolesFromOption<O> | undefined;
          };
        };
      };
    }, {
      error: null;
      success: boolean;
    }>;
  };
  $ERROR_CODES: {
    readonly FAILED_TO_CREATE_USER: "Failed to create user";
    readonly USER_ALREADY_EXISTS: "User already exists.";
    readonly USER_ALREADY_EXISTS_USE_ANOTHER_EMAIL: "User already exists. Use another email.";
    readonly YOU_CANNOT_BAN_YOURSELF: "You cannot ban yourself";
    readonly YOU_ARE_NOT_ALLOWED_TO_CHANGE_USERS_ROLE: "You are not allowed to change users role";
    readonly YOU_ARE_NOT_ALLOWED_TO_CREATE_USERS: "You are not allowed to create users";
    readonly YOU_ARE_NOT_ALLOWED_TO_LIST_USERS: "You are not allowed to list users";
    readonly YOU_ARE_NOT_ALLOWED_TO_LIST_USERS_SESSIONS: "You are not allowed to list users sessions";
    readonly YOU_ARE_NOT_ALLOWED_TO_BAN_USERS: "You are not allowed to ban users";
    readonly YOU_ARE_NOT_ALLOWED_TO_IMPERSONATE_USERS: "You are not allowed to impersonate users";
    readonly YOU_ARE_NOT_ALLOWED_TO_REVOKE_USERS_SESSIONS: "You are not allowed to revoke users sessions";
    readonly YOU_ARE_NOT_ALLOWED_TO_DELETE_USERS: "You are not allowed to delete users";
    readonly YOU_ARE_NOT_ALLOWED_TO_SET_USERS_PASSWORD: "You are not allowed to set users password";
    readonly BANNED_USER: "You have been banned from this application";
    readonly YOU_ARE_NOT_ALLOWED_TO_GET_USER: "You are not allowed to get user";
    readonly NO_DATA_TO_UPDATE: "No data to update";
    readonly YOU_ARE_NOT_ALLOWED_TO_UPDATE_USERS: "You are not allowed to update users";
    readonly YOU_CANNOT_REMOVE_YOURSELF: "You cannot remove yourself";
    readonly YOU_ARE_NOT_ALLOWED_TO_SET_NON_EXISTENT_VALUE: "You are not allowed to set a non-existent role value";
    readonly YOU_CANNOT_IMPERSONATE_ADMINS: "You cannot impersonate admins";
    readonly INVALID_ROLE_TYPE: "Invalid role type";
  };
  schema: {
    user: {
      fields: {
        role: {
          type: "string";
          required: false;
          input: false;
        };
        banned: {
          type: "boolean";
          defaultValue: false;
          required: false;
          input: false;
        };
        banReason: {
          type: "string";
          required: false;
          input: false;
        };
        banExpires: {
          type: "date";
          required: false;
          input: false;
        };
      };
    };
    session: {
      fields: {
        impersonatedBy: {
          type: "string";
          required: false;
        };
      };
    };
  };
  options: any;
};
//#endregion
export { admin };
//# sourceMappingURL=admin.d.mts.map