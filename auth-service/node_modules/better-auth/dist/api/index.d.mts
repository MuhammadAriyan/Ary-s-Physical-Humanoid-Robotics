import { InferFieldsFromOptions, InferFieldsFromPlugins } from "../db/field.mjs";
import "../db/index.mjs";
import { Prettify as Prettify$1, StripEmptyObjects, UnionToIntersection } from "../types/helper.mjs";
import { AdditionalSessionFieldsOutput, AdditionalUserFieldsInput, AdditionalUserFieldsOutput } from "../types/models.mjs";
import { Auth } from "../types/auth.mjs";
import "../types/index.mjs";
import "../index.mjs";
import { getIp } from "../utils/get-request-ip.mjs";
import { getOAuthState } from "./middlewares/oauth.mjs";
import { originCheck, originCheckMiddleware } from "./middlewares/origin-check.mjs";
import "./middlewares/index.mjs";
import { accountInfo, getAccessToken, linkSocialAccount, listUserAccounts, refreshToken, unlinkAccount } from "./routes/account.mjs";
import { callbackOAuth } from "./routes/callback.mjs";
import { createEmailVerificationToken, sendVerificationEmail, sendVerificationEmailFn, verifyEmail } from "./routes/email-verification.mjs";
import { error } from "./routes/error.mjs";
import { ok } from "./routes/ok.mjs";
import { requestPasswordReset, requestPasswordResetCallback, resetPassword } from "./routes/reset-password.mjs";
import { freshSessionMiddleware, getSession, getSessionFromCtx, listSessions, requestOnlySessionMiddleware, revokeOtherSessions, revokeSession, revokeSessions, sensitiveSessionMiddleware, sessionMiddleware } from "./routes/session.mjs";
import { signInEmail, signInSocial } from "./routes/sign-in.mjs";
import { signOut } from "./routes/sign-out.mjs";
import { signUpEmail } from "./routes/sign-up.mjs";
import { changeEmail, changePassword, deleteUser, deleteUserCallback, setPassword, updateUser } from "./routes/update-user.mjs";
import "./routes/index.mjs";
import { AuthContext, BetterAuthOptions, BetterAuthPlugin } from "@better-auth/core";
import { InternalLogger } from "@better-auth/core/env";
import * as _better_auth_core_oauth20 from "@better-auth/core/oauth2";
import * as zod80 from "zod";
import * as better_call110 from "better-call";
import { APIError } from "better-call";
import { AuthEndpoint, AuthMiddleware, createAuthEndpoint, createAuthMiddleware, optionsMiddleware } from "@better-auth/core/api";
import * as zod_v4_core15 from "zod/v4/core";

//#region src/api/index.d.ts
declare function checkEndpointConflicts(options: BetterAuthOptions, logger: InternalLogger): void;
declare function getEndpoints<Option extends BetterAuthOptions>(ctx: Promise<AuthContext> | AuthContext, options: Option): {
  api: {
    readonly ok: better_call110.StrictEndpoint<"/ok", {
      method: "GET";
      metadata: {
        openapi: {
          description: string;
          responses: {
            "200": {
              description: string;
              content: {
                "application/json": {
                  schema: {
                    type: "object";
                    properties: {
                      ok: {
                        type: string;
                        description: string;
                      };
                    };
                    required: string[];
                  };
                };
              };
            };
          };
        };
        scope: "server";
      };
    }, {
      ok: boolean;
    }>;
    readonly error: better_call110.StrictEndpoint<"/error", {
      method: "GET";
      metadata: {
        openapi: {
          description: string;
          responses: {
            "200": {
              description: string;
              content: {
                "text/html": {
                  schema: {
                    type: "string";
                    description: string;
                  };
                };
              };
            };
          };
        };
        scope: "server";
      };
    }, Response>;
    readonly signInSocial: better_call110.StrictEndpoint<"/sign-in/social", {
      method: "POST";
      operationId: string;
      body: zod80.ZodObject<{
        callbackURL: zod80.ZodOptional<zod80.ZodString>;
        newUserCallbackURL: zod80.ZodOptional<zod80.ZodString>;
        errorCallbackURL: zod80.ZodOptional<zod80.ZodString>;
        provider: zod80.ZodType<"github" | "apple" | "atlassian" | "cognito" | "discord" | "facebook" | "figma" | "microsoft" | "google" | "huggingface" | "slack" | "spotify" | "twitch" | "twitter" | "dropbox" | "kick" | "linear" | "linkedin" | "gitlab" | "tiktok" | "reddit" | "roblox" | "salesforce" | "vk" | "zoom" | "notion" | "kakao" | "naver" | "line" | "paybin" | "paypal" | "polar" | "vercel" | (string & {}), unknown, zod_v4_core15.$ZodTypeInternals<"github" | "apple" | "atlassian" | "cognito" | "discord" | "facebook" | "figma" | "microsoft" | "google" | "huggingface" | "slack" | "spotify" | "twitch" | "twitter" | "dropbox" | "kick" | "linear" | "linkedin" | "gitlab" | "tiktok" | "reddit" | "roblox" | "salesforce" | "vk" | "zoom" | "notion" | "kakao" | "naver" | "line" | "paybin" | "paypal" | "polar" | "vercel" | (string & {}), unknown>>;
        disableRedirect: zod80.ZodOptional<zod80.ZodBoolean>;
        idToken: zod80.ZodOptional<zod80.ZodObject<{
          token: zod80.ZodString;
          nonce: zod80.ZodOptional<zod80.ZodString>;
          accessToken: zod80.ZodOptional<zod80.ZodString>;
          refreshToken: zod80.ZodOptional<zod80.ZodString>;
          expiresAt: zod80.ZodOptional<zod80.ZodNumber>;
        }, zod_v4_core15.$strip>>;
        scopes: zod80.ZodOptional<zod80.ZodArray<zod80.ZodString>>;
        requestSignUp: zod80.ZodOptional<zod80.ZodBoolean>;
        loginHint: zod80.ZodOptional<zod80.ZodString>;
        additionalData: zod80.ZodOptional<zod80.ZodRecord<zod80.ZodString, zod80.ZodAny>>;
      }, zod_v4_core15.$strip>;
      metadata: {
        $Infer: {
          body: zod80.infer<zod80.ZodObject<{
            callbackURL: zod80.ZodOptional<zod80.ZodString>;
            newUserCallbackURL: zod80.ZodOptional<zod80.ZodString>;
            errorCallbackURL: zod80.ZodOptional<zod80.ZodString>;
            provider: zod80.ZodType<"github" | "apple" | "atlassian" | "cognito" | "discord" | "facebook" | "figma" | "microsoft" | "google" | "huggingface" | "slack" | "spotify" | "twitch" | "twitter" | "dropbox" | "kick" | "linear" | "linkedin" | "gitlab" | "tiktok" | "reddit" | "roblox" | "salesforce" | "vk" | "zoom" | "notion" | "kakao" | "naver" | "line" | "paybin" | "paypal" | "polar" | "vercel" | (string & {}), unknown, zod_v4_core15.$ZodTypeInternals<"github" | "apple" | "atlassian" | "cognito" | "discord" | "facebook" | "figma" | "microsoft" | "google" | "huggingface" | "slack" | "spotify" | "twitch" | "twitter" | "dropbox" | "kick" | "linear" | "linkedin" | "gitlab" | "tiktok" | "reddit" | "roblox" | "salesforce" | "vk" | "zoom" | "notion" | "kakao" | "naver" | "line" | "paybin" | "paypal" | "polar" | "vercel" | (string & {}), unknown>>;
            disableRedirect: zod80.ZodOptional<zod80.ZodBoolean>;
            idToken: zod80.ZodOptional<zod80.ZodObject<{
              token: zod80.ZodString;
              nonce: zod80.ZodOptional<zod80.ZodString>;
              accessToken: zod80.ZodOptional<zod80.ZodString>;
              refreshToken: zod80.ZodOptional<zod80.ZodString>;
              expiresAt: zod80.ZodOptional<zod80.ZodNumber>;
            }, zod_v4_core15.$strip>>;
            scopes: zod80.ZodOptional<zod80.ZodArray<zod80.ZodString>>;
            requestSignUp: zod80.ZodOptional<zod80.ZodBoolean>;
            loginHint: zod80.ZodOptional<zod80.ZodString>;
            additionalData: zod80.ZodOptional<zod80.ZodRecord<zod80.ZodString, zod80.ZodAny>>;
          }, zod_v4_core15.$strip>>;
          returned: {
            redirect: boolean;
            token?: string | undefined;
            url?: string | undefined;
            user?: UnionToIntersection<StripEmptyObjects<{
              id: string;
              createdAt: Date;
              updatedAt: Date;
              email: string;
              emailVerified: boolean;
              name: string;
              image?: string | null | undefined;
            } & (Option extends BetterAuthOptions ? AdditionalUserFieldsOutput<Option> : Option extends Auth ? AdditionalUserFieldsOutput<Option["options"]> : {})>> | undefined;
          };
        };
        openapi: {
          description: string;
          operationId: string;
          responses: {
            "200": {
              description: string;
              content: {
                "application/json": {
                  schema: {
                    type: "object";
                    description: string;
                    properties: {
                      token: {
                        type: string;
                      };
                      user: {
                        type: string;
                        $ref: string;
                      };
                      url: {
                        type: string;
                      };
                      redirect: {
                        type: string;
                        enum: boolean[];
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
      redirect: boolean;
      url: string;
    } | {
      redirect: boolean;
      token: string;
      url: undefined;
      user: UnionToIntersection<StripEmptyObjects<{
        id: string;
        createdAt: Date;
        updatedAt: Date;
        email: string;
        emailVerified: boolean;
        name: string;
        image?: string | null | undefined;
      } & (Option extends BetterAuthOptions ? AdditionalUserFieldsOutput<Option> : Option extends Auth ? AdditionalUserFieldsOutput<Option["options"]> : {})>>;
    }>;
    readonly callbackOAuth: better_call110.StrictEndpoint<"/callback/:id", {
      method: ("GET" | "POST")[];
      operationId: string;
      body: zod80.ZodOptional<zod80.ZodObject<{
        code: zod80.ZodOptional<zod80.ZodString>;
        error: zod80.ZodOptional<zod80.ZodString>;
        device_id: zod80.ZodOptional<zod80.ZodString>;
        error_description: zod80.ZodOptional<zod80.ZodString>;
        state: zod80.ZodOptional<zod80.ZodString>;
        user: zod80.ZodOptional<zod80.ZodString>;
      }, zod_v4_core15.$strip>>;
      query: zod80.ZodOptional<zod80.ZodObject<{
        code: zod80.ZodOptional<zod80.ZodString>;
        error: zod80.ZodOptional<zod80.ZodString>;
        device_id: zod80.ZodOptional<zod80.ZodString>;
        error_description: zod80.ZodOptional<zod80.ZodString>;
        state: zod80.ZodOptional<zod80.ZodString>;
        user: zod80.ZodOptional<zod80.ZodString>;
      }, zod_v4_core15.$strip>>;
      metadata: {
        allowedMediaTypes: string[];
        scope: "server";
      };
    }, void>;
    readonly getSession: better_call110.StrictEndpoint<"/get-session", {
      method: "GET";
      operationId: string;
      query: zod80.ZodOptional<zod80.ZodObject<{
        disableCookieCache: zod80.ZodOptional<zod80.ZodCoercedBoolean<unknown>>;
        disableRefresh: zod80.ZodOptional<zod80.ZodCoercedBoolean<unknown>>;
      }, zod_v4_core15.$strip>>;
      requireHeaders: true;
      metadata: {
        openapi: {
          operationId: string;
          description: string;
          responses: {
            "200": {
              description: string;
              content: {
                "application/json": {
                  schema: {
                    type: "object";
                    nullable: boolean;
                    properties: {
                      session: {
                        $ref: string;
                      };
                      user: {
                        $ref: string;
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
      session: UnionToIntersection<StripEmptyObjects<{
        id: string;
        createdAt: Date;
        updatedAt: Date;
        userId: string;
        expiresAt: Date;
        token: string;
        ipAddress?: string | null | undefined;
        userAgent?: string | null | undefined;
      } & (Option extends BetterAuthOptions ? AdditionalSessionFieldsOutput<Option> : Option extends Auth ? AdditionalSessionFieldsOutput<Option["options"]> : {})>>;
      user: UnionToIntersection<StripEmptyObjects<{
        id: string;
        createdAt: Date;
        updatedAt: Date;
        email: string;
        emailVerified: boolean;
        name: string;
        image?: string | null | undefined;
      } & (Option extends BetterAuthOptions ? AdditionalUserFieldsOutput<Option> : Option extends Auth ? AdditionalUserFieldsOutput<Option["options"]> : {})>>;
    } | null>;
    readonly signOut: better_call110.StrictEndpoint<"/sign-out", {
      method: "POST";
      operationId: string;
      requireHeaders: true;
      metadata: {
        openapi: {
          operationId: string;
          description: string;
          responses: {
            "200": {
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
    readonly signUpEmail: better_call110.StrictEndpoint<"/sign-up/email", {
      method: "POST";
      operationId: string;
      body: zod80.ZodIntersection<zod80.ZodObject<{
        name: zod80.ZodString;
        email: zod80.ZodEmail;
        password: zod80.ZodString;
        image: zod80.ZodOptional<zod80.ZodString>;
        callbackURL: zod80.ZodOptional<zod80.ZodString>;
        rememberMe: zod80.ZodOptional<zod80.ZodBoolean>;
      }, zod_v4_core15.$strip>, zod80.ZodRecord<zod80.ZodString, zod80.ZodAny>>;
      metadata: {
        $Infer: {
          body: {
            name: string;
            email: string;
            password: string;
            image?: string | undefined;
            callbackURL?: string | undefined;
            rememberMe?: boolean | undefined;
          } & InferFieldsFromPlugins<Option, "user", "input"> & InferFieldsFromOptions<Option, "user", "input">;
          returned: {
            token: string | null;
            user: UnionToIntersection<StripEmptyObjects<{
              id: string;
              createdAt: Date;
              updatedAt: Date;
              email: string;
              emailVerified: boolean;
              name: string;
              image?: string | null | undefined;
            } & (Option extends BetterAuthOptions ? AdditionalUserFieldsOutput<Option> : Option extends Auth ? AdditionalUserFieldsOutput<Option["options"]> : {})>>;
          };
        };
        openapi: {
          operationId: string;
          description: string;
          requestBody: {
            content: {
              "application/json": {
                schema: {
                  type: "object";
                  properties: {
                    name: {
                      type: string;
                      description: string;
                    };
                    email: {
                      type: string;
                      description: string;
                    };
                    password: {
                      type: string;
                      description: string;
                    };
                    image: {
                      type: string;
                      description: string;
                    };
                    callbackURL: {
                      type: string;
                      description: string;
                    };
                    rememberMe: {
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
                      token: {
                        type: string;
                        nullable: boolean;
                        description: string;
                      };
                      user: {
                        type: string;
                        properties: {
                          id: {
                            type: string;
                            description: string;
                          };
                          email: {
                            type: string;
                            format: string;
                            description: string;
                          };
                          name: {
                            type: string;
                            description: string;
                          };
                          image: {
                            type: string;
                            format: string;
                            nullable: boolean;
                            description: string;
                          };
                          emailVerified: {
                            type: string;
                            description: string;
                          };
                          createdAt: {
                            type: string;
                            format: string;
                            description: string;
                          };
                          updatedAt: {
                            type: string;
                            format: string;
                            description: string;
                          };
                        };
                        required: string[];
                      };
                    };
                    required: string[];
                  };
                };
              };
            };
            "422": {
              description: string;
              content: {
                "application/json": {
                  schema: {
                    type: "object";
                    properties: {
                      message: {
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
      token: null;
      user: UnionToIntersection<StripEmptyObjects<{
        id: string;
        createdAt: Date;
        updatedAt: Date;
        email: string;
        emailVerified: boolean;
        name: string;
        image?: string | null | undefined;
      } & (Option extends BetterAuthOptions ? AdditionalUserFieldsOutput<Option> : Option extends Auth ? AdditionalUserFieldsOutput<Option["options"]> : {})>>;
    } | {
      token: string;
      user: UnionToIntersection<StripEmptyObjects<{
        id: string;
        createdAt: Date;
        updatedAt: Date;
        email: string;
        emailVerified: boolean;
        name: string;
        image?: string | null | undefined;
      } & (Option extends BetterAuthOptions ? AdditionalUserFieldsOutput<Option> : Option extends Auth ? AdditionalUserFieldsOutput<Option["options"]> : {})>>;
    }>;
    readonly signInEmail: better_call110.StrictEndpoint<"/sign-in/email", {
      method: "POST";
      operationId: string;
      body: zod80.ZodObject<{
        email: zod80.ZodString;
        password: zod80.ZodString;
        callbackURL: zod80.ZodOptional<zod80.ZodString>;
        rememberMe: zod80.ZodOptional<zod80.ZodDefault<zod80.ZodBoolean>>;
      }, zod_v4_core15.$strip>;
      metadata: {
        $Infer: {
          body: {
            email: string;
            password: string;
            callbackURL?: string | undefined;
            rememberMe?: boolean | undefined;
          };
          returned: {
            redirect: boolean;
            token: string;
            url?: string | undefined;
            user: UnionToIntersection<StripEmptyObjects<{
              id: string;
              createdAt: Date;
              updatedAt: Date;
              email: string;
              emailVerified: boolean;
              name: string;
              image?: string | null | undefined;
            } & (Option extends BetterAuthOptions ? AdditionalUserFieldsOutput<Option> : Option extends Auth ? AdditionalUserFieldsOutput<Option["options"]> : {})>>;
          };
        };
        openapi: {
          operationId: string;
          description: string;
          responses: {
            "200": {
              description: string;
              content: {
                "application/json": {
                  schema: {
                    type: "object";
                    description: string;
                    properties: {
                      redirect: {
                        type: string;
                        enum: boolean[];
                      };
                      token: {
                        type: string;
                        description: string;
                      };
                      url: {
                        type: string;
                        nullable: boolean;
                      };
                      user: {
                        type: string;
                        $ref: string;
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
      redirect: boolean;
      token: string;
      url?: string | undefined;
      user: UnionToIntersection<StripEmptyObjects<{
        id: string;
        createdAt: Date;
        updatedAt: Date;
        email: string;
        emailVerified: boolean;
        name: string;
        image?: string | null | undefined;
      } & (Option extends BetterAuthOptions ? AdditionalUserFieldsOutput<Option> : Option extends Auth ? AdditionalUserFieldsOutput<Option["options"]> : {})>>;
    }>;
    readonly resetPassword: better_call110.StrictEndpoint<"/reset-password", {
      method: "POST";
      operationId: string;
      query: zod80.ZodOptional<zod80.ZodObject<{
        token: zod80.ZodOptional<zod80.ZodString>;
      }, zod_v4_core15.$strip>>;
      body: zod80.ZodObject<{
        newPassword: zod80.ZodString;
        token: zod80.ZodOptional<zod80.ZodString>;
      }, zod_v4_core15.$strip>;
      metadata: {
        openapi: {
          operationId: string;
          description: string;
          responses: {
            "200": {
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
    readonly verifyEmail: better_call110.StrictEndpoint<"/verify-email", {
      method: "GET";
      operationId: string;
      query: zod80.ZodObject<{
        token: zod80.ZodString;
        callbackURL: zod80.ZodOptional<zod80.ZodString>;
      }, zod_v4_core15.$strip>;
      use: ((inputContext: better_call110.MiddlewareInputContext<better_call110.MiddlewareOptions>) => Promise<void>)[];
      metadata: {
        openapi: {
          description: string;
          parameters: ({
            name: string;
            in: "query";
            description: string;
            required: true;
            schema: {
              type: "string";
            };
          } | {
            name: string;
            in: "query";
            description: string;
            required: false;
            schema: {
              type: "string";
            };
          })[];
          responses: {
            "200": {
              description: string;
              content: {
                "application/json": {
                  schema: {
                    type: "object";
                    properties: {
                      user: {
                        type: string;
                        $ref: string;
                      };
                      status: {
                        type: string;
                        description: string;
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
    }, void | {
      status: boolean;
    }>;
    readonly sendVerificationEmail: better_call110.StrictEndpoint<"/send-verification-email", {
      method: "POST";
      operationId: string;
      body: zod80.ZodObject<{
        email: zod80.ZodEmail;
        callbackURL: zod80.ZodOptional<zod80.ZodString>;
      }, zod_v4_core15.$strip>;
      metadata: {
        openapi: {
          operationId: string;
          description: string;
          requestBody: {
            content: {
              "application/json": {
                schema: {
                  type: "object";
                  properties: {
                    email: {
                      type: string;
                      description: string;
                      example: string;
                    };
                    callbackURL: {
                      type: string;
                      description: string;
                      example: string;
                      nullable: boolean;
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
                      status: {
                        type: string;
                        description: string;
                        example: boolean;
                      };
                    };
                  };
                };
              };
            };
            "400": {
              description: string;
              content: {
                "application/json": {
                  schema: {
                    type: "object";
                    properties: {
                      message: {
                        type: string;
                        description: string;
                        example: string;
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
    readonly changeEmail: better_call110.StrictEndpoint<"/change-email", {
      method: "POST";
      body: zod80.ZodObject<{
        newEmail: zod80.ZodEmail;
        callbackURL: zod80.ZodOptional<zod80.ZodString>;
      }, zod_v4_core15.$strip>;
      use: ((inputContext: better_call110.MiddlewareInputContext<better_call110.MiddlewareOptions>) => Promise<{
        session: {
          session: Record<string, any> & {
            id: string;
            createdAt: Date;
            updatedAt: Date;
            userId: string;
            expiresAt: Date;
            token: string;
            ipAddress?: string | null | undefined;
            userAgent?: string | null | undefined;
          };
          user: Record<string, any> & {
            id: string;
            createdAt: Date;
            updatedAt: Date;
            email: string;
            emailVerified: boolean;
            name: string;
            image?: string | null | undefined;
          };
        };
      }>)[];
      metadata: {
        openapi: {
          operationId: string;
          responses: {
            "200": {
              description: string;
              content: {
                "application/json": {
                  schema: {
                    type: "object";
                    properties: {
                      user: {
                        type: string;
                        $ref: string;
                      };
                      status: {
                        type: string;
                        description: string;
                      };
                      message: {
                        type: string;
                        enum: string[];
                        description: string;
                        nullable: boolean;
                      };
                    };
                    required: string[];
                  };
                };
              };
            };
            "422": {
              description: string;
              content: {
                "application/json": {
                  schema: {
                    type: "object";
                    properties: {
                      message: {
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
    readonly changePassword: better_call110.StrictEndpoint<"/change-password", {
      method: "POST";
      operationId: string;
      body: zod80.ZodObject<{
        newPassword: zod80.ZodString;
        currentPassword: zod80.ZodString;
        revokeOtherSessions: zod80.ZodOptional<zod80.ZodBoolean>;
      }, zod_v4_core15.$strip>;
      use: ((inputContext: better_call110.MiddlewareInputContext<better_call110.MiddlewareOptions>) => Promise<{
        session: {
          session: Record<string, any> & {
            id: string;
            createdAt: Date;
            updatedAt: Date;
            userId: string;
            expiresAt: Date;
            token: string;
            ipAddress?: string | null | undefined;
            userAgent?: string | null | undefined;
          };
          user: Record<string, any> & {
            id: string;
            createdAt: Date;
            updatedAt: Date;
            email: string;
            emailVerified: boolean;
            name: string;
            image?: string | null | undefined;
          };
        };
      }>)[];
      metadata: {
        openapi: {
          operationId: string;
          description: string;
          responses: {
            "200": {
              description: string;
              content: {
                "application/json": {
                  schema: {
                    type: "object";
                    properties: {
                      token: {
                        type: string;
                        nullable: boolean;
                        description: string;
                      };
                      user: {
                        type: string;
                        properties: {
                          id: {
                            type: string;
                            description: string;
                          };
                          email: {
                            type: string;
                            format: string;
                            description: string;
                          };
                          name: {
                            type: string;
                            description: string;
                          };
                          image: {
                            type: string;
                            format: string;
                            nullable: boolean;
                            description: string;
                          };
                          emailVerified: {
                            type: string;
                            description: string;
                          };
                          createdAt: {
                            type: string;
                            format: string;
                            description: string;
                          };
                          updatedAt: {
                            type: string;
                            format: string;
                            description: string;
                          };
                        };
                        required: string[];
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
      token: string | null;
      user: {
        id: string;
        email: string;
        name: string;
        image: string | null | undefined;
        emailVerified: boolean;
        createdAt: Date;
        updatedAt: Date;
      };
    }>;
    readonly setPassword: better_call110.StrictEndpoint<string, {
      method: "POST";
      body: zod80.ZodObject<{
        newPassword: zod80.ZodString;
      }, zod_v4_core15.$strip>;
      use: ((inputContext: better_call110.MiddlewareInputContext<better_call110.MiddlewareOptions>) => Promise<{
        session: {
          session: Record<string, any> & {
            id: string;
            createdAt: Date;
            updatedAt: Date;
            userId: string;
            expiresAt: Date;
            token: string;
            ipAddress?: string | null | undefined;
            userAgent?: string | null | undefined;
          };
          user: Record<string, any> & {
            id: string;
            createdAt: Date;
            updatedAt: Date;
            email: string;
            emailVerified: boolean;
            name: string;
            image?: string | null | undefined;
          };
        };
      }>)[];
    }, {
      status: boolean;
    }>;
    readonly updateUser: better_call110.StrictEndpoint<"/update-user", {
      method: "POST";
      operationId: string;
      body: zod80.ZodRecord<zod80.ZodString, zod80.ZodAny>;
      use: ((inputContext: better_call110.MiddlewareInputContext<better_call110.MiddlewareOptions>) => Promise<{
        session: {
          session: Record<string, any> & {
            id: string;
            createdAt: Date;
            updatedAt: Date;
            userId: string;
            expiresAt: Date;
            token: string;
            ipAddress?: string | null | undefined;
            userAgent?: string | null | undefined;
          };
          user: Record<string, any> & {
            id: string;
            createdAt: Date;
            updatedAt: Date;
            email: string;
            emailVerified: boolean;
            name: string;
            image?: string | null | undefined;
          };
        };
      }>)[];
      metadata: {
        $Infer: {
          body: Partial<AdditionalUserFieldsInput<Option>> & {
            name?: string | undefined;
            image?: string | undefined | null;
          };
        };
        openapi: {
          operationId: string;
          description: string;
          requestBody: {
            content: {
              "application/json": {
                schema: {
                  type: "object";
                  properties: {
                    name: {
                      type: string;
                      description: string;
                    };
                    image: {
                      type: string;
                      description: string;
                      nullable: boolean;
                    };
                  };
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
                      user: {
                        type: string;
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
      status: boolean;
    }>;
    readonly deleteUser: better_call110.StrictEndpoint<"/delete-user", {
      method: "POST";
      use: ((inputContext: better_call110.MiddlewareInputContext<better_call110.MiddlewareOptions>) => Promise<{
        session: {
          session: Record<string, any> & {
            id: string;
            createdAt: Date;
            updatedAt: Date;
            userId: string;
            expiresAt: Date;
            token: string;
            ipAddress?: string | null | undefined;
            userAgent?: string | null | undefined;
          };
          user: Record<string, any> & {
            id: string;
            createdAt: Date;
            updatedAt: Date;
            email: string;
            emailVerified: boolean;
            name: string;
            image?: string | null | undefined;
          };
        };
      }>)[];
      body: zod80.ZodObject<{
        callbackURL: zod80.ZodOptional<zod80.ZodString>;
        password: zod80.ZodOptional<zod80.ZodString>;
        token: zod80.ZodOptional<zod80.ZodString>;
      }, zod_v4_core15.$strip>;
      metadata: {
        openapi: {
          operationId: string;
          description: string;
          requestBody: {
            content: {
              "application/json": {
                schema: {
                  type: "object";
                  properties: {
                    callbackURL: {
                      type: string;
                      description: string;
                    };
                    password: {
                      type: string;
                      description: string;
                    };
                    token: {
                      type: string;
                      description: string;
                    };
                  };
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
                      success: {
                        type: string;
                        description: string;
                      };
                      message: {
                        type: string;
                        enum: string[];
                        description: string;
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
      success: boolean;
      message: string;
    }>;
    readonly requestPasswordReset: better_call110.StrictEndpoint<"/request-password-reset", {
      method: "POST";
      body: zod80.ZodObject<{
        email: zod80.ZodEmail;
        redirectTo: zod80.ZodOptional<zod80.ZodString>;
      }, zod_v4_core15.$strip>;
      metadata: {
        openapi: {
          operationId: string;
          description: string;
          responses: {
            "200": {
              description: string;
              content: {
                "application/json": {
                  schema: {
                    type: "object";
                    properties: {
                      status: {
                        type: string;
                      };
                      message: {
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
      message: string;
    }>;
    readonly requestPasswordResetCallback: better_call110.StrictEndpoint<"/reset-password/:token", {
      method: "GET";
      operationId: string;
      query: zod80.ZodObject<{
        callbackURL: zod80.ZodString;
      }, zod_v4_core15.$strip>;
      use: ((inputContext: better_call110.MiddlewareInputContext<better_call110.MiddlewareOptions>) => Promise<void>)[];
      metadata: {
        openapi: {
          operationId: string;
          description: string;
          parameters: ({
            name: string;
            in: "path";
            required: true;
            description: string;
            schema: {
              type: "string";
            };
          } | {
            name: string;
            in: "query";
            required: true;
            description: string;
            schema: {
              type: "string";
            };
          })[];
          responses: {
            "200": {
              description: string;
              content: {
                "application/json": {
                  schema: {
                    type: "object";
                    properties: {
                      token: {
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
    }, never>;
    readonly listSessions: better_call110.StrictEndpoint<"/list-sessions", {
      method: "GET";
      operationId: string;
      use: ((inputContext: better_call110.MiddlewareInputContext<better_call110.MiddlewareOptions>) => Promise<{
        session: {
          session: Record<string, any> & {
            id: string;
            createdAt: Date;
            updatedAt: Date;
            userId: string;
            expiresAt: Date;
            token: string;
            ipAddress?: string | null | undefined;
            userAgent?: string | null | undefined;
          };
          user: Record<string, any> & {
            id: string;
            createdAt: Date;
            updatedAt: Date;
            email: string;
            emailVerified: boolean;
            name: string;
            image?: string | null | undefined;
          };
        };
      }>)[];
      requireHeaders: true;
      metadata: {
        openapi: {
          operationId: string;
          description: string;
          responses: {
            "200": {
              description: string;
              content: {
                "application/json": {
                  schema: {
                    type: "array";
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
    }, Prettify$1<UnionToIntersection<StripEmptyObjects<{
      id: string;
      createdAt: Date;
      updatedAt: Date;
      userId: string;
      expiresAt: Date;
      token: string;
      ipAddress?: string | null | undefined;
      userAgent?: string | null | undefined;
    } & (Option extends BetterAuthOptions ? AdditionalSessionFieldsOutput<Option> : Option extends Auth ? AdditionalSessionFieldsOutput<Option["options"]> : {})>>>[]>;
    readonly revokeSession: better_call110.StrictEndpoint<"/revoke-session", {
      method: "POST";
      body: zod80.ZodObject<{
        token: zod80.ZodString;
      }, zod_v4_core15.$strip>;
      use: ((inputContext: better_call110.MiddlewareInputContext<better_call110.MiddlewareOptions>) => Promise<{
        session: {
          session: Record<string, any> & {
            id: string;
            createdAt: Date;
            updatedAt: Date;
            userId: string;
            expiresAt: Date;
            token: string;
            ipAddress?: string | null | undefined;
            userAgent?: string | null | undefined;
          };
          user: Record<string, any> & {
            id: string;
            createdAt: Date;
            updatedAt: Date;
            email: string;
            emailVerified: boolean;
            name: string;
            image?: string | null | undefined;
          };
        };
      }>)[];
      requireHeaders: true;
      metadata: {
        openapi: {
          description: string;
          requestBody: {
            content: {
              "application/json": {
                schema: {
                  type: "object";
                  properties: {
                    token: {
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
                      status: {
                        type: string;
                        description: string;
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
      status: boolean;
    }>;
    readonly revokeSessions: better_call110.StrictEndpoint<"/revoke-sessions", {
      method: "POST";
      use: ((inputContext: better_call110.MiddlewareInputContext<better_call110.MiddlewareOptions>) => Promise<{
        session: {
          session: Record<string, any> & {
            id: string;
            createdAt: Date;
            updatedAt: Date;
            userId: string;
            expiresAt: Date;
            token: string;
            ipAddress?: string | null | undefined;
            userAgent?: string | null | undefined;
          };
          user: Record<string, any> & {
            id: string;
            createdAt: Date;
            updatedAt: Date;
            email: string;
            emailVerified: boolean;
            name: string;
            image?: string | null | undefined;
          };
        };
      }>)[];
      requireHeaders: true;
      metadata: {
        openapi: {
          description: string;
          responses: {
            "200": {
              description: string;
              content: {
                "application/json": {
                  schema: {
                    type: "object";
                    properties: {
                      status: {
                        type: string;
                        description: string;
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
      status: boolean;
    }>;
    readonly revokeOtherSessions: better_call110.StrictEndpoint<"/revoke-other-sessions", {
      method: "POST";
      requireHeaders: true;
      use: ((inputContext: better_call110.MiddlewareInputContext<better_call110.MiddlewareOptions>) => Promise<{
        session: {
          session: Record<string, any> & {
            id: string;
            createdAt: Date;
            updatedAt: Date;
            userId: string;
            expiresAt: Date;
            token: string;
            ipAddress?: string | null | undefined;
            userAgent?: string | null | undefined;
          };
          user: Record<string, any> & {
            id: string;
            createdAt: Date;
            updatedAt: Date;
            email: string;
            emailVerified: boolean;
            name: string;
            image?: string | null | undefined;
          };
        };
      }>)[];
      metadata: {
        openapi: {
          description: string;
          responses: {
            "200": {
              description: string;
              content: {
                "application/json": {
                  schema: {
                    type: "object";
                    properties: {
                      status: {
                        type: string;
                        description: string;
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
      status: boolean;
    }>;
    readonly linkSocialAccount: better_call110.StrictEndpoint<"/link-social", {
      method: "POST";
      requireHeaders: true;
      body: zod80.ZodObject<{
        callbackURL: zod80.ZodOptional<zod80.ZodString>;
        provider: zod80.ZodType<"github" | "apple" | "atlassian" | "cognito" | "discord" | "facebook" | "figma" | "microsoft" | "google" | "huggingface" | "slack" | "spotify" | "twitch" | "twitter" | "dropbox" | "kick" | "linear" | "linkedin" | "gitlab" | "tiktok" | "reddit" | "roblox" | "salesforce" | "vk" | "zoom" | "notion" | "kakao" | "naver" | "line" | "paybin" | "paypal" | "polar" | "vercel" | (string & {}), unknown, zod_v4_core15.$ZodTypeInternals<"github" | "apple" | "atlassian" | "cognito" | "discord" | "facebook" | "figma" | "microsoft" | "google" | "huggingface" | "slack" | "spotify" | "twitch" | "twitter" | "dropbox" | "kick" | "linear" | "linkedin" | "gitlab" | "tiktok" | "reddit" | "roblox" | "salesforce" | "vk" | "zoom" | "notion" | "kakao" | "naver" | "line" | "paybin" | "paypal" | "polar" | "vercel" | (string & {}), unknown>>;
        idToken: zod80.ZodOptional<zod80.ZodObject<{
          token: zod80.ZodString;
          nonce: zod80.ZodOptional<zod80.ZodString>;
          accessToken: zod80.ZodOptional<zod80.ZodString>;
          refreshToken: zod80.ZodOptional<zod80.ZodString>;
          scopes: zod80.ZodOptional<zod80.ZodArray<zod80.ZodString>>;
        }, zod_v4_core15.$strip>>;
        requestSignUp: zod80.ZodOptional<zod80.ZodBoolean>;
        scopes: zod80.ZodOptional<zod80.ZodArray<zod80.ZodString>>;
        errorCallbackURL: zod80.ZodOptional<zod80.ZodString>;
        disableRedirect: zod80.ZodOptional<zod80.ZodBoolean>;
        additionalData: zod80.ZodOptional<zod80.ZodRecord<zod80.ZodString, zod80.ZodAny>>;
      }, zod_v4_core15.$strip>;
      use: ((inputContext: better_call110.MiddlewareInputContext<better_call110.MiddlewareOptions>) => Promise<{
        session: {
          session: Record<string, any> & {
            id: string;
            createdAt: Date;
            updatedAt: Date;
            userId: string;
            expiresAt: Date;
            token: string;
            ipAddress?: string | null | undefined;
            userAgent?: string | null | undefined;
          };
          user: Record<string, any> & {
            id: string;
            createdAt: Date;
            updatedAt: Date;
            email: string;
            emailVerified: boolean;
            name: string;
            image?: string | null | undefined;
          };
        };
      }>)[];
      metadata: {
        openapi: {
          description: string;
          operationId: string;
          responses: {
            "200": {
              description: string;
              content: {
                "application/json": {
                  schema: {
                    type: "object";
                    properties: {
                      url: {
                        type: string;
                        description: string;
                      };
                      redirect: {
                        type: string;
                        description: string;
                      };
                      status: {
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
      url: string;
      redirect: boolean;
    }>;
    readonly listUserAccounts: better_call110.StrictEndpoint<"/list-accounts", {
      method: "GET";
      use: ((inputContext: better_call110.MiddlewareInputContext<better_call110.MiddlewareOptions>) => Promise<{
        session: {
          session: Record<string, any> & {
            id: string;
            createdAt: Date;
            updatedAt: Date;
            userId: string;
            expiresAt: Date;
            token: string;
            ipAddress?: string | null | undefined;
            userAgent?: string | null | undefined;
          };
          user: Record<string, any> & {
            id: string;
            createdAt: Date;
            updatedAt: Date;
            email: string;
            emailVerified: boolean;
            name: string;
            image?: string | null | undefined;
          };
        };
      }>)[];
      metadata: {
        openapi: {
          operationId: string;
          description: string;
          responses: {
            "200": {
              description: string;
              content: {
                "application/json": {
                  schema: {
                    type: "array";
                    items: {
                      type: string;
                      properties: {
                        id: {
                          type: string;
                        };
                        providerId: {
                          type: string;
                        };
                        createdAt: {
                          type: string;
                          format: string;
                        };
                        updatedAt: {
                          type: string;
                          format: string;
                        };
                        accountId: {
                          type: string;
                        };
                        userId: {
                          type: string;
                        };
                        scopes: {
                          type: string;
                          items: {
                            type: string;
                          };
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
      };
    }, {
      id: string;
      providerId: string;
      createdAt: Date;
      updatedAt: Date;
      accountId: string;
      userId: string;
      scopes: string[];
    }[]>;
    readonly deleteUserCallback: better_call110.StrictEndpoint<"/delete-user/callback", {
      method: "GET";
      query: zod80.ZodObject<{
        token: zod80.ZodString;
        callbackURL: zod80.ZodOptional<zod80.ZodString>;
      }, zod_v4_core15.$strip>;
      use: ((inputContext: better_call110.MiddlewareInputContext<better_call110.MiddlewareOptions>) => Promise<void>)[];
      metadata: {
        openapi: {
          description: string;
          responses: {
            "200": {
              description: string;
              content: {
                "application/json": {
                  schema: {
                    type: "object";
                    properties: {
                      success: {
                        type: string;
                        description: string;
                      };
                      message: {
                        type: string;
                        enum: string[];
                        description: string;
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
      success: boolean;
      message: string;
    }>;
    readonly unlinkAccount: better_call110.StrictEndpoint<"/unlink-account", {
      method: "POST";
      body: zod80.ZodObject<{
        providerId: zod80.ZodString;
        accountId: zod80.ZodOptional<zod80.ZodString>;
      }, zod_v4_core15.$strip>;
      use: ((inputContext: better_call110.MiddlewareInputContext<better_call110.MiddlewareOptions>) => Promise<{
        session: {
          session: Record<string, any> & {
            id: string;
            createdAt: Date;
            updatedAt: Date;
            userId: string;
            expiresAt: Date;
            token: string;
            ipAddress?: string | null | undefined;
            userAgent?: string | null | undefined;
          };
          user: Record<string, any> & {
            id: string;
            createdAt: Date;
            updatedAt: Date;
            email: string;
            emailVerified: boolean;
            name: string;
            image?: string | null | undefined;
          };
        };
      }>)[];
      metadata: {
        openapi: {
          description: string;
          responses: {
            "200": {
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
    readonly refreshToken: better_call110.StrictEndpoint<"/refresh-token", {
      method: "POST";
      body: zod80.ZodObject<{
        providerId: zod80.ZodString;
        accountId: zod80.ZodOptional<zod80.ZodString>;
        userId: zod80.ZodOptional<zod80.ZodString>;
      }, zod_v4_core15.$strip>;
      metadata: {
        openapi: {
          description: string;
          responses: {
            200: {
              description: string;
              content: {
                "application/json": {
                  schema: {
                    type: "object";
                    properties: {
                      tokenType: {
                        type: string;
                      };
                      idToken: {
                        type: string;
                      };
                      accessToken: {
                        type: string;
                      };
                      refreshToken: {
                        type: string;
                      };
                      accessTokenExpiresAt: {
                        type: string;
                        format: string;
                      };
                      refreshTokenExpiresAt: {
                        type: string;
                        format: string;
                      };
                    };
                  };
                };
              };
            };
            400: {
              description: string;
            };
          };
        };
      };
    }, {
      accessToken: string | undefined;
      refreshToken: string | undefined;
      accessTokenExpiresAt: Date | undefined;
      refreshTokenExpiresAt: Date | undefined;
      scope: string | null | undefined;
      idToken: string | null | undefined;
      providerId: string;
      accountId: string;
    }>;
    readonly getAccessToken: better_call110.StrictEndpoint<"/get-access-token", {
      method: "POST";
      body: zod80.ZodObject<{
        providerId: zod80.ZodString;
        accountId: zod80.ZodOptional<zod80.ZodString>;
        userId: zod80.ZodOptional<zod80.ZodString>;
      }, zod_v4_core15.$strip>;
      metadata: {
        openapi: {
          description: string;
          responses: {
            200: {
              description: string;
              content: {
                "application/json": {
                  schema: {
                    type: "object";
                    properties: {
                      tokenType: {
                        type: string;
                      };
                      idToken: {
                        type: string;
                      };
                      accessToken: {
                        type: string;
                      };
                      refreshToken: {
                        type: string;
                      };
                      accessTokenExpiresAt: {
                        type: string;
                        format: string;
                      };
                      refreshTokenExpiresAt: {
                        type: string;
                        format: string;
                      };
                    };
                  };
                };
              };
            };
            400: {
              description: string;
            };
          };
        };
      };
    }, {
      accessToken: string;
      accessTokenExpiresAt: Date | undefined;
      scopes: string[];
      idToken: string | undefined;
    }>;
    readonly accountInfo: better_call110.StrictEndpoint<"/account-info", {
      method: "GET";
      use: ((inputContext: better_call110.MiddlewareInputContext<better_call110.MiddlewareOptions>) => Promise<{
        session: {
          session: Record<string, any> & {
            id: string;
            createdAt: Date;
            updatedAt: Date;
            userId: string;
            expiresAt: Date;
            token: string;
            ipAddress?: string | null | undefined;
            userAgent?: string | null | undefined;
          };
          user: Record<string, any> & {
            id: string;
            createdAt: Date;
            updatedAt: Date;
            email: string;
            emailVerified: boolean;
            name: string;
            image?: string | null | undefined;
          };
        };
      }>)[];
      metadata: {
        openapi: {
          description: string;
          responses: {
            "200": {
              description: string;
              content: {
                "application/json": {
                  schema: {
                    type: "object";
                    properties: {
                      user: {
                        type: string;
                        properties: {
                          id: {
                            type: string;
                          };
                          name: {
                            type: string;
                          };
                          email: {
                            type: string;
                          };
                          image: {
                            type: string;
                          };
                          emailVerified: {
                            type: string;
                          };
                        };
                        required: string[];
                      };
                      data: {
                        type: string;
                        properties: {};
                        additionalProperties: boolean;
                      };
                    };
                    required: string[];
                    additionalProperties: boolean;
                  };
                };
              };
            };
          };
        };
      };
      query: zod80.ZodOptional<zod80.ZodObject<{
        accountId: zod80.ZodOptional<zod80.ZodString>;
      }, zod_v4_core15.$strip>>;
    }, {
      user: _better_auth_core_oauth20.OAuth2UserInfo;
      data: Record<string, any>;
    } | null>;
  } & UnionToIntersection<Option["plugins"] extends (infer T)[] ? T extends BetterAuthPlugin ? T extends {
    endpoints: infer E;
  } ? E : {} : {} : {}>;
  middlewares: {
    path: string;
    middleware: any;
  }[];
};
declare const router: <Option extends BetterAuthOptions>(ctx: AuthContext, options: Option) => {
  handler: (request: Request) => Promise<Response>;
  endpoints: {
    readonly ok: better_call110.StrictEndpoint<"/ok", {
      method: "GET";
      metadata: {
        openapi: {
          description: string;
          responses: {
            "200": {
              description: string;
              content: {
                "application/json": {
                  schema: {
                    type: "object";
                    properties: {
                      ok: {
                        type: string;
                        description: string;
                      };
                    };
                    required: string[];
                  };
                };
              };
            };
          };
        };
        scope: "server";
      };
    }, {
      ok: boolean;
    }>;
    readonly error: better_call110.StrictEndpoint<"/error", {
      method: "GET";
      metadata: {
        openapi: {
          description: string;
          responses: {
            "200": {
              description: string;
              content: {
                "text/html": {
                  schema: {
                    type: "string";
                    description: string;
                  };
                };
              };
            };
          };
        };
        scope: "server";
      };
    }, Response>;
    readonly signInSocial: better_call110.StrictEndpoint<"/sign-in/social", {
      method: "POST";
      operationId: string;
      body: zod80.ZodObject<{
        callbackURL: zod80.ZodOptional<zod80.ZodString>;
        newUserCallbackURL: zod80.ZodOptional<zod80.ZodString>;
        errorCallbackURL: zod80.ZodOptional<zod80.ZodString>;
        provider: zod80.ZodType<"github" | "apple" | "atlassian" | "cognito" | "discord" | "facebook" | "figma" | "microsoft" | "google" | "huggingface" | "slack" | "spotify" | "twitch" | "twitter" | "dropbox" | "kick" | "linear" | "linkedin" | "gitlab" | "tiktok" | "reddit" | "roblox" | "salesforce" | "vk" | "zoom" | "notion" | "kakao" | "naver" | "line" | "paybin" | "paypal" | "polar" | "vercel" | (string & {}), unknown, zod_v4_core15.$ZodTypeInternals<"github" | "apple" | "atlassian" | "cognito" | "discord" | "facebook" | "figma" | "microsoft" | "google" | "huggingface" | "slack" | "spotify" | "twitch" | "twitter" | "dropbox" | "kick" | "linear" | "linkedin" | "gitlab" | "tiktok" | "reddit" | "roblox" | "salesforce" | "vk" | "zoom" | "notion" | "kakao" | "naver" | "line" | "paybin" | "paypal" | "polar" | "vercel" | (string & {}), unknown>>;
        disableRedirect: zod80.ZodOptional<zod80.ZodBoolean>;
        idToken: zod80.ZodOptional<zod80.ZodObject<{
          token: zod80.ZodString;
          nonce: zod80.ZodOptional<zod80.ZodString>;
          accessToken: zod80.ZodOptional<zod80.ZodString>;
          refreshToken: zod80.ZodOptional<zod80.ZodString>;
          expiresAt: zod80.ZodOptional<zod80.ZodNumber>;
        }, zod_v4_core15.$strip>>;
        scopes: zod80.ZodOptional<zod80.ZodArray<zod80.ZodString>>;
        requestSignUp: zod80.ZodOptional<zod80.ZodBoolean>;
        loginHint: zod80.ZodOptional<zod80.ZodString>;
        additionalData: zod80.ZodOptional<zod80.ZodRecord<zod80.ZodString, zod80.ZodAny>>;
      }, zod_v4_core15.$strip>;
      metadata: {
        $Infer: {
          body: zod80.infer<zod80.ZodObject<{
            callbackURL: zod80.ZodOptional<zod80.ZodString>;
            newUserCallbackURL: zod80.ZodOptional<zod80.ZodString>;
            errorCallbackURL: zod80.ZodOptional<zod80.ZodString>;
            provider: zod80.ZodType<"github" | "apple" | "atlassian" | "cognito" | "discord" | "facebook" | "figma" | "microsoft" | "google" | "huggingface" | "slack" | "spotify" | "twitch" | "twitter" | "dropbox" | "kick" | "linear" | "linkedin" | "gitlab" | "tiktok" | "reddit" | "roblox" | "salesforce" | "vk" | "zoom" | "notion" | "kakao" | "naver" | "line" | "paybin" | "paypal" | "polar" | "vercel" | (string & {}), unknown, zod_v4_core15.$ZodTypeInternals<"github" | "apple" | "atlassian" | "cognito" | "discord" | "facebook" | "figma" | "microsoft" | "google" | "huggingface" | "slack" | "spotify" | "twitch" | "twitter" | "dropbox" | "kick" | "linear" | "linkedin" | "gitlab" | "tiktok" | "reddit" | "roblox" | "salesforce" | "vk" | "zoom" | "notion" | "kakao" | "naver" | "line" | "paybin" | "paypal" | "polar" | "vercel" | (string & {}), unknown>>;
            disableRedirect: zod80.ZodOptional<zod80.ZodBoolean>;
            idToken: zod80.ZodOptional<zod80.ZodObject<{
              token: zod80.ZodString;
              nonce: zod80.ZodOptional<zod80.ZodString>;
              accessToken: zod80.ZodOptional<zod80.ZodString>;
              refreshToken: zod80.ZodOptional<zod80.ZodString>;
              expiresAt: zod80.ZodOptional<zod80.ZodNumber>;
            }, zod_v4_core15.$strip>>;
            scopes: zod80.ZodOptional<zod80.ZodArray<zod80.ZodString>>;
            requestSignUp: zod80.ZodOptional<zod80.ZodBoolean>;
            loginHint: zod80.ZodOptional<zod80.ZodString>;
            additionalData: zod80.ZodOptional<zod80.ZodRecord<zod80.ZodString, zod80.ZodAny>>;
          }, zod_v4_core15.$strip>>;
          returned: {
            redirect: boolean;
            token?: string | undefined;
            url?: string | undefined;
            user?: UnionToIntersection<StripEmptyObjects<{
              id: string;
              createdAt: Date;
              updatedAt: Date;
              email: string;
              emailVerified: boolean;
              name: string;
              image?: string | null | undefined;
            } & (Option extends BetterAuthOptions ? AdditionalUserFieldsOutput<Option> : Option extends Auth ? AdditionalUserFieldsOutput<Option["options"]> : {})>> | undefined;
          };
        };
        openapi: {
          description: string;
          operationId: string;
          responses: {
            "200": {
              description: string;
              content: {
                "application/json": {
                  schema: {
                    type: "object";
                    description: string;
                    properties: {
                      token: {
                        type: string;
                      };
                      user: {
                        type: string;
                        $ref: string;
                      };
                      url: {
                        type: string;
                      };
                      redirect: {
                        type: string;
                        enum: boolean[];
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
      redirect: boolean;
      url: string;
    } | {
      redirect: boolean;
      token: string;
      url: undefined;
      user: UnionToIntersection<StripEmptyObjects<{
        id: string;
        createdAt: Date;
        updatedAt: Date;
        email: string;
        emailVerified: boolean;
        name: string;
        image?: string | null | undefined;
      } & (Option extends BetterAuthOptions ? AdditionalUserFieldsOutput<Option> : Option extends Auth ? AdditionalUserFieldsOutput<Option["options"]> : {})>>;
    }>;
    readonly callbackOAuth: better_call110.StrictEndpoint<"/callback/:id", {
      method: ("GET" | "POST")[];
      operationId: string;
      body: zod80.ZodOptional<zod80.ZodObject<{
        code: zod80.ZodOptional<zod80.ZodString>;
        error: zod80.ZodOptional<zod80.ZodString>;
        device_id: zod80.ZodOptional<zod80.ZodString>;
        error_description: zod80.ZodOptional<zod80.ZodString>;
        state: zod80.ZodOptional<zod80.ZodString>;
        user: zod80.ZodOptional<zod80.ZodString>;
      }, zod_v4_core15.$strip>>;
      query: zod80.ZodOptional<zod80.ZodObject<{
        code: zod80.ZodOptional<zod80.ZodString>;
        error: zod80.ZodOptional<zod80.ZodString>;
        device_id: zod80.ZodOptional<zod80.ZodString>;
        error_description: zod80.ZodOptional<zod80.ZodString>;
        state: zod80.ZodOptional<zod80.ZodString>;
        user: zod80.ZodOptional<zod80.ZodString>;
      }, zod_v4_core15.$strip>>;
      metadata: {
        allowedMediaTypes: string[];
        scope: "server";
      };
    }, void>;
    readonly getSession: better_call110.StrictEndpoint<"/get-session", {
      method: "GET";
      operationId: string;
      query: zod80.ZodOptional<zod80.ZodObject<{
        disableCookieCache: zod80.ZodOptional<zod80.ZodCoercedBoolean<unknown>>;
        disableRefresh: zod80.ZodOptional<zod80.ZodCoercedBoolean<unknown>>;
      }, zod_v4_core15.$strip>>;
      requireHeaders: true;
      metadata: {
        openapi: {
          operationId: string;
          description: string;
          responses: {
            "200": {
              description: string;
              content: {
                "application/json": {
                  schema: {
                    type: "object";
                    nullable: boolean;
                    properties: {
                      session: {
                        $ref: string;
                      };
                      user: {
                        $ref: string;
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
      session: UnionToIntersection<StripEmptyObjects<{
        id: string;
        createdAt: Date;
        updatedAt: Date;
        userId: string;
        expiresAt: Date;
        token: string;
        ipAddress?: string | null | undefined;
        userAgent?: string | null | undefined;
      } & (Option extends BetterAuthOptions ? AdditionalSessionFieldsOutput<Option> : Option extends Auth ? AdditionalSessionFieldsOutput<Option["options"]> : {})>>;
      user: UnionToIntersection<StripEmptyObjects<{
        id: string;
        createdAt: Date;
        updatedAt: Date;
        email: string;
        emailVerified: boolean;
        name: string;
        image?: string | null | undefined;
      } & (Option extends BetterAuthOptions ? AdditionalUserFieldsOutput<Option> : Option extends Auth ? AdditionalUserFieldsOutput<Option["options"]> : {})>>;
    } | null>;
    readonly signOut: better_call110.StrictEndpoint<"/sign-out", {
      method: "POST";
      operationId: string;
      requireHeaders: true;
      metadata: {
        openapi: {
          operationId: string;
          description: string;
          responses: {
            "200": {
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
    readonly signUpEmail: better_call110.StrictEndpoint<"/sign-up/email", {
      method: "POST";
      operationId: string;
      body: zod80.ZodIntersection<zod80.ZodObject<{
        name: zod80.ZodString;
        email: zod80.ZodEmail;
        password: zod80.ZodString;
        image: zod80.ZodOptional<zod80.ZodString>;
        callbackURL: zod80.ZodOptional<zod80.ZodString>;
        rememberMe: zod80.ZodOptional<zod80.ZodBoolean>;
      }, zod_v4_core15.$strip>, zod80.ZodRecord<zod80.ZodString, zod80.ZodAny>>;
      metadata: {
        $Infer: {
          body: {
            name: string;
            email: string;
            password: string;
            image?: string | undefined;
            callbackURL?: string | undefined;
            rememberMe?: boolean | undefined;
          } & InferFieldsFromPlugins<Option, "user", "input"> & InferFieldsFromOptions<Option, "user", "input">;
          returned: {
            token: string | null;
            user: UnionToIntersection<StripEmptyObjects<{
              id: string;
              createdAt: Date;
              updatedAt: Date;
              email: string;
              emailVerified: boolean;
              name: string;
              image?: string | null | undefined;
            } & (Option extends BetterAuthOptions ? AdditionalUserFieldsOutput<Option> : Option extends Auth ? AdditionalUserFieldsOutput<Option["options"]> : {})>>;
          };
        };
        openapi: {
          operationId: string;
          description: string;
          requestBody: {
            content: {
              "application/json": {
                schema: {
                  type: "object";
                  properties: {
                    name: {
                      type: string;
                      description: string;
                    };
                    email: {
                      type: string;
                      description: string;
                    };
                    password: {
                      type: string;
                      description: string;
                    };
                    image: {
                      type: string;
                      description: string;
                    };
                    callbackURL: {
                      type: string;
                      description: string;
                    };
                    rememberMe: {
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
                      token: {
                        type: string;
                        nullable: boolean;
                        description: string;
                      };
                      user: {
                        type: string;
                        properties: {
                          id: {
                            type: string;
                            description: string;
                          };
                          email: {
                            type: string;
                            format: string;
                            description: string;
                          };
                          name: {
                            type: string;
                            description: string;
                          };
                          image: {
                            type: string;
                            format: string;
                            nullable: boolean;
                            description: string;
                          };
                          emailVerified: {
                            type: string;
                            description: string;
                          };
                          createdAt: {
                            type: string;
                            format: string;
                            description: string;
                          };
                          updatedAt: {
                            type: string;
                            format: string;
                            description: string;
                          };
                        };
                        required: string[];
                      };
                    };
                    required: string[];
                  };
                };
              };
            };
            "422": {
              description: string;
              content: {
                "application/json": {
                  schema: {
                    type: "object";
                    properties: {
                      message: {
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
      token: null;
      user: UnionToIntersection<StripEmptyObjects<{
        id: string;
        createdAt: Date;
        updatedAt: Date;
        email: string;
        emailVerified: boolean;
        name: string;
        image?: string | null | undefined;
      } & (Option extends BetterAuthOptions ? AdditionalUserFieldsOutput<Option> : Option extends Auth ? AdditionalUserFieldsOutput<Option["options"]> : {})>>;
    } | {
      token: string;
      user: UnionToIntersection<StripEmptyObjects<{
        id: string;
        createdAt: Date;
        updatedAt: Date;
        email: string;
        emailVerified: boolean;
        name: string;
        image?: string | null | undefined;
      } & (Option extends BetterAuthOptions ? AdditionalUserFieldsOutput<Option> : Option extends Auth ? AdditionalUserFieldsOutput<Option["options"]> : {})>>;
    }>;
    readonly signInEmail: better_call110.StrictEndpoint<"/sign-in/email", {
      method: "POST";
      operationId: string;
      body: zod80.ZodObject<{
        email: zod80.ZodString;
        password: zod80.ZodString;
        callbackURL: zod80.ZodOptional<zod80.ZodString>;
        rememberMe: zod80.ZodOptional<zod80.ZodDefault<zod80.ZodBoolean>>;
      }, zod_v4_core15.$strip>;
      metadata: {
        $Infer: {
          body: {
            email: string;
            password: string;
            callbackURL?: string | undefined;
            rememberMe?: boolean | undefined;
          };
          returned: {
            redirect: boolean;
            token: string;
            url?: string | undefined;
            user: UnionToIntersection<StripEmptyObjects<{
              id: string;
              createdAt: Date;
              updatedAt: Date;
              email: string;
              emailVerified: boolean;
              name: string;
              image?: string | null | undefined;
            } & (Option extends BetterAuthOptions ? AdditionalUserFieldsOutput<Option> : Option extends Auth ? AdditionalUserFieldsOutput<Option["options"]> : {})>>;
          };
        };
        openapi: {
          operationId: string;
          description: string;
          responses: {
            "200": {
              description: string;
              content: {
                "application/json": {
                  schema: {
                    type: "object";
                    description: string;
                    properties: {
                      redirect: {
                        type: string;
                        enum: boolean[];
                      };
                      token: {
                        type: string;
                        description: string;
                      };
                      url: {
                        type: string;
                        nullable: boolean;
                      };
                      user: {
                        type: string;
                        $ref: string;
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
      redirect: boolean;
      token: string;
      url?: string | undefined;
      user: UnionToIntersection<StripEmptyObjects<{
        id: string;
        createdAt: Date;
        updatedAt: Date;
        email: string;
        emailVerified: boolean;
        name: string;
        image?: string | null | undefined;
      } & (Option extends BetterAuthOptions ? AdditionalUserFieldsOutput<Option> : Option extends Auth ? AdditionalUserFieldsOutput<Option["options"]> : {})>>;
    }>;
    readonly resetPassword: better_call110.StrictEndpoint<"/reset-password", {
      method: "POST";
      operationId: string;
      query: zod80.ZodOptional<zod80.ZodObject<{
        token: zod80.ZodOptional<zod80.ZodString>;
      }, zod_v4_core15.$strip>>;
      body: zod80.ZodObject<{
        newPassword: zod80.ZodString;
        token: zod80.ZodOptional<zod80.ZodString>;
      }, zod_v4_core15.$strip>;
      metadata: {
        openapi: {
          operationId: string;
          description: string;
          responses: {
            "200": {
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
    readonly verifyEmail: better_call110.StrictEndpoint<"/verify-email", {
      method: "GET";
      operationId: string;
      query: zod80.ZodObject<{
        token: zod80.ZodString;
        callbackURL: zod80.ZodOptional<zod80.ZodString>;
      }, zod_v4_core15.$strip>;
      use: ((inputContext: better_call110.MiddlewareInputContext<better_call110.MiddlewareOptions>) => Promise<void>)[];
      metadata: {
        openapi: {
          description: string;
          parameters: ({
            name: string;
            in: "query";
            description: string;
            required: true;
            schema: {
              type: "string";
            };
          } | {
            name: string;
            in: "query";
            description: string;
            required: false;
            schema: {
              type: "string";
            };
          })[];
          responses: {
            "200": {
              description: string;
              content: {
                "application/json": {
                  schema: {
                    type: "object";
                    properties: {
                      user: {
                        type: string;
                        $ref: string;
                      };
                      status: {
                        type: string;
                        description: string;
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
    }, void | {
      status: boolean;
    }>;
    readonly sendVerificationEmail: better_call110.StrictEndpoint<"/send-verification-email", {
      method: "POST";
      operationId: string;
      body: zod80.ZodObject<{
        email: zod80.ZodEmail;
        callbackURL: zod80.ZodOptional<zod80.ZodString>;
      }, zod_v4_core15.$strip>;
      metadata: {
        openapi: {
          operationId: string;
          description: string;
          requestBody: {
            content: {
              "application/json": {
                schema: {
                  type: "object";
                  properties: {
                    email: {
                      type: string;
                      description: string;
                      example: string;
                    };
                    callbackURL: {
                      type: string;
                      description: string;
                      example: string;
                      nullable: boolean;
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
                      status: {
                        type: string;
                        description: string;
                        example: boolean;
                      };
                    };
                  };
                };
              };
            };
            "400": {
              description: string;
              content: {
                "application/json": {
                  schema: {
                    type: "object";
                    properties: {
                      message: {
                        type: string;
                        description: string;
                        example: string;
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
    readonly changeEmail: better_call110.StrictEndpoint<"/change-email", {
      method: "POST";
      body: zod80.ZodObject<{
        newEmail: zod80.ZodEmail;
        callbackURL: zod80.ZodOptional<zod80.ZodString>;
      }, zod_v4_core15.$strip>;
      use: ((inputContext: better_call110.MiddlewareInputContext<better_call110.MiddlewareOptions>) => Promise<{
        session: {
          session: Record<string, any> & {
            id: string;
            createdAt: Date;
            updatedAt: Date;
            userId: string;
            expiresAt: Date;
            token: string;
            ipAddress?: string | null | undefined;
            userAgent?: string | null | undefined;
          };
          user: Record<string, any> & {
            id: string;
            createdAt: Date;
            updatedAt: Date;
            email: string;
            emailVerified: boolean;
            name: string;
            image?: string | null | undefined;
          };
        };
      }>)[];
      metadata: {
        openapi: {
          operationId: string;
          responses: {
            "200": {
              description: string;
              content: {
                "application/json": {
                  schema: {
                    type: "object";
                    properties: {
                      user: {
                        type: string;
                        $ref: string;
                      };
                      status: {
                        type: string;
                        description: string;
                      };
                      message: {
                        type: string;
                        enum: string[];
                        description: string;
                        nullable: boolean;
                      };
                    };
                    required: string[];
                  };
                };
              };
            };
            "422": {
              description: string;
              content: {
                "application/json": {
                  schema: {
                    type: "object";
                    properties: {
                      message: {
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
    readonly changePassword: better_call110.StrictEndpoint<"/change-password", {
      method: "POST";
      operationId: string;
      body: zod80.ZodObject<{
        newPassword: zod80.ZodString;
        currentPassword: zod80.ZodString;
        revokeOtherSessions: zod80.ZodOptional<zod80.ZodBoolean>;
      }, zod_v4_core15.$strip>;
      use: ((inputContext: better_call110.MiddlewareInputContext<better_call110.MiddlewareOptions>) => Promise<{
        session: {
          session: Record<string, any> & {
            id: string;
            createdAt: Date;
            updatedAt: Date;
            userId: string;
            expiresAt: Date;
            token: string;
            ipAddress?: string | null | undefined;
            userAgent?: string | null | undefined;
          };
          user: Record<string, any> & {
            id: string;
            createdAt: Date;
            updatedAt: Date;
            email: string;
            emailVerified: boolean;
            name: string;
            image?: string | null | undefined;
          };
        };
      }>)[];
      metadata: {
        openapi: {
          operationId: string;
          description: string;
          responses: {
            "200": {
              description: string;
              content: {
                "application/json": {
                  schema: {
                    type: "object";
                    properties: {
                      token: {
                        type: string;
                        nullable: boolean;
                        description: string;
                      };
                      user: {
                        type: string;
                        properties: {
                          id: {
                            type: string;
                            description: string;
                          };
                          email: {
                            type: string;
                            format: string;
                            description: string;
                          };
                          name: {
                            type: string;
                            description: string;
                          };
                          image: {
                            type: string;
                            format: string;
                            nullable: boolean;
                            description: string;
                          };
                          emailVerified: {
                            type: string;
                            description: string;
                          };
                          createdAt: {
                            type: string;
                            format: string;
                            description: string;
                          };
                          updatedAt: {
                            type: string;
                            format: string;
                            description: string;
                          };
                        };
                        required: string[];
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
      token: string | null;
      user: {
        id: string;
        email: string;
        name: string;
        image: string | null | undefined;
        emailVerified: boolean;
        createdAt: Date;
        updatedAt: Date;
      };
    }>;
    readonly setPassword: better_call110.StrictEndpoint<string, {
      method: "POST";
      body: zod80.ZodObject<{
        newPassword: zod80.ZodString;
      }, zod_v4_core15.$strip>;
      use: ((inputContext: better_call110.MiddlewareInputContext<better_call110.MiddlewareOptions>) => Promise<{
        session: {
          session: Record<string, any> & {
            id: string;
            createdAt: Date;
            updatedAt: Date;
            userId: string;
            expiresAt: Date;
            token: string;
            ipAddress?: string | null | undefined;
            userAgent?: string | null | undefined;
          };
          user: Record<string, any> & {
            id: string;
            createdAt: Date;
            updatedAt: Date;
            email: string;
            emailVerified: boolean;
            name: string;
            image?: string | null | undefined;
          };
        };
      }>)[];
    }, {
      status: boolean;
    }>;
    readonly updateUser: better_call110.StrictEndpoint<"/update-user", {
      method: "POST";
      operationId: string;
      body: zod80.ZodRecord<zod80.ZodString, zod80.ZodAny>;
      use: ((inputContext: better_call110.MiddlewareInputContext<better_call110.MiddlewareOptions>) => Promise<{
        session: {
          session: Record<string, any> & {
            id: string;
            createdAt: Date;
            updatedAt: Date;
            userId: string;
            expiresAt: Date;
            token: string;
            ipAddress?: string | null | undefined;
            userAgent?: string | null | undefined;
          };
          user: Record<string, any> & {
            id: string;
            createdAt: Date;
            updatedAt: Date;
            email: string;
            emailVerified: boolean;
            name: string;
            image?: string | null | undefined;
          };
        };
      }>)[];
      metadata: {
        $Infer: {
          body: Partial<AdditionalUserFieldsInput<Option>> & {
            name?: string | undefined;
            image?: string | undefined | null;
          };
        };
        openapi: {
          operationId: string;
          description: string;
          requestBody: {
            content: {
              "application/json": {
                schema: {
                  type: "object";
                  properties: {
                    name: {
                      type: string;
                      description: string;
                    };
                    image: {
                      type: string;
                      description: string;
                      nullable: boolean;
                    };
                  };
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
                      user: {
                        type: string;
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
      status: boolean;
    }>;
    readonly deleteUser: better_call110.StrictEndpoint<"/delete-user", {
      method: "POST";
      use: ((inputContext: better_call110.MiddlewareInputContext<better_call110.MiddlewareOptions>) => Promise<{
        session: {
          session: Record<string, any> & {
            id: string;
            createdAt: Date;
            updatedAt: Date;
            userId: string;
            expiresAt: Date;
            token: string;
            ipAddress?: string | null | undefined;
            userAgent?: string | null | undefined;
          };
          user: Record<string, any> & {
            id: string;
            createdAt: Date;
            updatedAt: Date;
            email: string;
            emailVerified: boolean;
            name: string;
            image?: string | null | undefined;
          };
        };
      }>)[];
      body: zod80.ZodObject<{
        callbackURL: zod80.ZodOptional<zod80.ZodString>;
        password: zod80.ZodOptional<zod80.ZodString>;
        token: zod80.ZodOptional<zod80.ZodString>;
      }, zod_v4_core15.$strip>;
      metadata: {
        openapi: {
          operationId: string;
          description: string;
          requestBody: {
            content: {
              "application/json": {
                schema: {
                  type: "object";
                  properties: {
                    callbackURL: {
                      type: string;
                      description: string;
                    };
                    password: {
                      type: string;
                      description: string;
                    };
                    token: {
                      type: string;
                      description: string;
                    };
                  };
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
                      success: {
                        type: string;
                        description: string;
                      };
                      message: {
                        type: string;
                        enum: string[];
                        description: string;
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
      success: boolean;
      message: string;
    }>;
    readonly requestPasswordReset: better_call110.StrictEndpoint<"/request-password-reset", {
      method: "POST";
      body: zod80.ZodObject<{
        email: zod80.ZodEmail;
        redirectTo: zod80.ZodOptional<zod80.ZodString>;
      }, zod_v4_core15.$strip>;
      metadata: {
        openapi: {
          operationId: string;
          description: string;
          responses: {
            "200": {
              description: string;
              content: {
                "application/json": {
                  schema: {
                    type: "object";
                    properties: {
                      status: {
                        type: string;
                      };
                      message: {
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
      message: string;
    }>;
    readonly requestPasswordResetCallback: better_call110.StrictEndpoint<"/reset-password/:token", {
      method: "GET";
      operationId: string;
      query: zod80.ZodObject<{
        callbackURL: zod80.ZodString;
      }, zod_v4_core15.$strip>;
      use: ((inputContext: better_call110.MiddlewareInputContext<better_call110.MiddlewareOptions>) => Promise<void>)[];
      metadata: {
        openapi: {
          operationId: string;
          description: string;
          parameters: ({
            name: string;
            in: "path";
            required: true;
            description: string;
            schema: {
              type: "string";
            };
          } | {
            name: string;
            in: "query";
            required: true;
            description: string;
            schema: {
              type: "string";
            };
          })[];
          responses: {
            "200": {
              description: string;
              content: {
                "application/json": {
                  schema: {
                    type: "object";
                    properties: {
                      token: {
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
    }, never>;
    readonly listSessions: better_call110.StrictEndpoint<"/list-sessions", {
      method: "GET";
      operationId: string;
      use: ((inputContext: better_call110.MiddlewareInputContext<better_call110.MiddlewareOptions>) => Promise<{
        session: {
          session: Record<string, any> & {
            id: string;
            createdAt: Date;
            updatedAt: Date;
            userId: string;
            expiresAt: Date;
            token: string;
            ipAddress?: string | null | undefined;
            userAgent?: string | null | undefined;
          };
          user: Record<string, any> & {
            id: string;
            createdAt: Date;
            updatedAt: Date;
            email: string;
            emailVerified: boolean;
            name: string;
            image?: string | null | undefined;
          };
        };
      }>)[];
      requireHeaders: true;
      metadata: {
        openapi: {
          operationId: string;
          description: string;
          responses: {
            "200": {
              description: string;
              content: {
                "application/json": {
                  schema: {
                    type: "array";
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
    }, Prettify$1<UnionToIntersection<StripEmptyObjects<{
      id: string;
      createdAt: Date;
      updatedAt: Date;
      userId: string;
      expiresAt: Date;
      token: string;
      ipAddress?: string | null | undefined;
      userAgent?: string | null | undefined;
    } & (Option extends BetterAuthOptions ? AdditionalSessionFieldsOutput<Option> : Option extends Auth ? AdditionalSessionFieldsOutput<Option["options"]> : {})>>>[]>;
    readonly revokeSession: better_call110.StrictEndpoint<"/revoke-session", {
      method: "POST";
      body: zod80.ZodObject<{
        token: zod80.ZodString;
      }, zod_v4_core15.$strip>;
      use: ((inputContext: better_call110.MiddlewareInputContext<better_call110.MiddlewareOptions>) => Promise<{
        session: {
          session: Record<string, any> & {
            id: string;
            createdAt: Date;
            updatedAt: Date;
            userId: string;
            expiresAt: Date;
            token: string;
            ipAddress?: string | null | undefined;
            userAgent?: string | null | undefined;
          };
          user: Record<string, any> & {
            id: string;
            createdAt: Date;
            updatedAt: Date;
            email: string;
            emailVerified: boolean;
            name: string;
            image?: string | null | undefined;
          };
        };
      }>)[];
      requireHeaders: true;
      metadata: {
        openapi: {
          description: string;
          requestBody: {
            content: {
              "application/json": {
                schema: {
                  type: "object";
                  properties: {
                    token: {
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
                      status: {
                        type: string;
                        description: string;
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
      status: boolean;
    }>;
    readonly revokeSessions: better_call110.StrictEndpoint<"/revoke-sessions", {
      method: "POST";
      use: ((inputContext: better_call110.MiddlewareInputContext<better_call110.MiddlewareOptions>) => Promise<{
        session: {
          session: Record<string, any> & {
            id: string;
            createdAt: Date;
            updatedAt: Date;
            userId: string;
            expiresAt: Date;
            token: string;
            ipAddress?: string | null | undefined;
            userAgent?: string | null | undefined;
          };
          user: Record<string, any> & {
            id: string;
            createdAt: Date;
            updatedAt: Date;
            email: string;
            emailVerified: boolean;
            name: string;
            image?: string | null | undefined;
          };
        };
      }>)[];
      requireHeaders: true;
      metadata: {
        openapi: {
          description: string;
          responses: {
            "200": {
              description: string;
              content: {
                "application/json": {
                  schema: {
                    type: "object";
                    properties: {
                      status: {
                        type: string;
                        description: string;
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
      status: boolean;
    }>;
    readonly revokeOtherSessions: better_call110.StrictEndpoint<"/revoke-other-sessions", {
      method: "POST";
      requireHeaders: true;
      use: ((inputContext: better_call110.MiddlewareInputContext<better_call110.MiddlewareOptions>) => Promise<{
        session: {
          session: Record<string, any> & {
            id: string;
            createdAt: Date;
            updatedAt: Date;
            userId: string;
            expiresAt: Date;
            token: string;
            ipAddress?: string | null | undefined;
            userAgent?: string | null | undefined;
          };
          user: Record<string, any> & {
            id: string;
            createdAt: Date;
            updatedAt: Date;
            email: string;
            emailVerified: boolean;
            name: string;
            image?: string | null | undefined;
          };
        };
      }>)[];
      metadata: {
        openapi: {
          description: string;
          responses: {
            "200": {
              description: string;
              content: {
                "application/json": {
                  schema: {
                    type: "object";
                    properties: {
                      status: {
                        type: string;
                        description: string;
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
      status: boolean;
    }>;
    readonly linkSocialAccount: better_call110.StrictEndpoint<"/link-social", {
      method: "POST";
      requireHeaders: true;
      body: zod80.ZodObject<{
        callbackURL: zod80.ZodOptional<zod80.ZodString>;
        provider: zod80.ZodType<"github" | "apple" | "atlassian" | "cognito" | "discord" | "facebook" | "figma" | "microsoft" | "google" | "huggingface" | "slack" | "spotify" | "twitch" | "twitter" | "dropbox" | "kick" | "linear" | "linkedin" | "gitlab" | "tiktok" | "reddit" | "roblox" | "salesforce" | "vk" | "zoom" | "notion" | "kakao" | "naver" | "line" | "paybin" | "paypal" | "polar" | "vercel" | (string & {}), unknown, zod_v4_core15.$ZodTypeInternals<"github" | "apple" | "atlassian" | "cognito" | "discord" | "facebook" | "figma" | "microsoft" | "google" | "huggingface" | "slack" | "spotify" | "twitch" | "twitter" | "dropbox" | "kick" | "linear" | "linkedin" | "gitlab" | "tiktok" | "reddit" | "roblox" | "salesforce" | "vk" | "zoom" | "notion" | "kakao" | "naver" | "line" | "paybin" | "paypal" | "polar" | "vercel" | (string & {}), unknown>>;
        idToken: zod80.ZodOptional<zod80.ZodObject<{
          token: zod80.ZodString;
          nonce: zod80.ZodOptional<zod80.ZodString>;
          accessToken: zod80.ZodOptional<zod80.ZodString>;
          refreshToken: zod80.ZodOptional<zod80.ZodString>;
          scopes: zod80.ZodOptional<zod80.ZodArray<zod80.ZodString>>;
        }, zod_v4_core15.$strip>>;
        requestSignUp: zod80.ZodOptional<zod80.ZodBoolean>;
        scopes: zod80.ZodOptional<zod80.ZodArray<zod80.ZodString>>;
        errorCallbackURL: zod80.ZodOptional<zod80.ZodString>;
        disableRedirect: zod80.ZodOptional<zod80.ZodBoolean>;
        additionalData: zod80.ZodOptional<zod80.ZodRecord<zod80.ZodString, zod80.ZodAny>>;
      }, zod_v4_core15.$strip>;
      use: ((inputContext: better_call110.MiddlewareInputContext<better_call110.MiddlewareOptions>) => Promise<{
        session: {
          session: Record<string, any> & {
            id: string;
            createdAt: Date;
            updatedAt: Date;
            userId: string;
            expiresAt: Date;
            token: string;
            ipAddress?: string | null | undefined;
            userAgent?: string | null | undefined;
          };
          user: Record<string, any> & {
            id: string;
            createdAt: Date;
            updatedAt: Date;
            email: string;
            emailVerified: boolean;
            name: string;
            image?: string | null | undefined;
          };
        };
      }>)[];
      metadata: {
        openapi: {
          description: string;
          operationId: string;
          responses: {
            "200": {
              description: string;
              content: {
                "application/json": {
                  schema: {
                    type: "object";
                    properties: {
                      url: {
                        type: string;
                        description: string;
                      };
                      redirect: {
                        type: string;
                        description: string;
                      };
                      status: {
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
      url: string;
      redirect: boolean;
    }>;
    readonly listUserAccounts: better_call110.StrictEndpoint<"/list-accounts", {
      method: "GET";
      use: ((inputContext: better_call110.MiddlewareInputContext<better_call110.MiddlewareOptions>) => Promise<{
        session: {
          session: Record<string, any> & {
            id: string;
            createdAt: Date;
            updatedAt: Date;
            userId: string;
            expiresAt: Date;
            token: string;
            ipAddress?: string | null | undefined;
            userAgent?: string | null | undefined;
          };
          user: Record<string, any> & {
            id: string;
            createdAt: Date;
            updatedAt: Date;
            email: string;
            emailVerified: boolean;
            name: string;
            image?: string | null | undefined;
          };
        };
      }>)[];
      metadata: {
        openapi: {
          operationId: string;
          description: string;
          responses: {
            "200": {
              description: string;
              content: {
                "application/json": {
                  schema: {
                    type: "array";
                    items: {
                      type: string;
                      properties: {
                        id: {
                          type: string;
                        };
                        providerId: {
                          type: string;
                        };
                        createdAt: {
                          type: string;
                          format: string;
                        };
                        updatedAt: {
                          type: string;
                          format: string;
                        };
                        accountId: {
                          type: string;
                        };
                        userId: {
                          type: string;
                        };
                        scopes: {
                          type: string;
                          items: {
                            type: string;
                          };
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
      };
    }, {
      id: string;
      providerId: string;
      createdAt: Date;
      updatedAt: Date;
      accountId: string;
      userId: string;
      scopes: string[];
    }[]>;
    readonly deleteUserCallback: better_call110.StrictEndpoint<"/delete-user/callback", {
      method: "GET";
      query: zod80.ZodObject<{
        token: zod80.ZodString;
        callbackURL: zod80.ZodOptional<zod80.ZodString>;
      }, zod_v4_core15.$strip>;
      use: ((inputContext: better_call110.MiddlewareInputContext<better_call110.MiddlewareOptions>) => Promise<void>)[];
      metadata: {
        openapi: {
          description: string;
          responses: {
            "200": {
              description: string;
              content: {
                "application/json": {
                  schema: {
                    type: "object";
                    properties: {
                      success: {
                        type: string;
                        description: string;
                      };
                      message: {
                        type: string;
                        enum: string[];
                        description: string;
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
      success: boolean;
      message: string;
    }>;
    readonly unlinkAccount: better_call110.StrictEndpoint<"/unlink-account", {
      method: "POST";
      body: zod80.ZodObject<{
        providerId: zod80.ZodString;
        accountId: zod80.ZodOptional<zod80.ZodString>;
      }, zod_v4_core15.$strip>;
      use: ((inputContext: better_call110.MiddlewareInputContext<better_call110.MiddlewareOptions>) => Promise<{
        session: {
          session: Record<string, any> & {
            id: string;
            createdAt: Date;
            updatedAt: Date;
            userId: string;
            expiresAt: Date;
            token: string;
            ipAddress?: string | null | undefined;
            userAgent?: string | null | undefined;
          };
          user: Record<string, any> & {
            id: string;
            createdAt: Date;
            updatedAt: Date;
            email: string;
            emailVerified: boolean;
            name: string;
            image?: string | null | undefined;
          };
        };
      }>)[];
      metadata: {
        openapi: {
          description: string;
          responses: {
            "200": {
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
    readonly refreshToken: better_call110.StrictEndpoint<"/refresh-token", {
      method: "POST";
      body: zod80.ZodObject<{
        providerId: zod80.ZodString;
        accountId: zod80.ZodOptional<zod80.ZodString>;
        userId: zod80.ZodOptional<zod80.ZodString>;
      }, zod_v4_core15.$strip>;
      metadata: {
        openapi: {
          description: string;
          responses: {
            200: {
              description: string;
              content: {
                "application/json": {
                  schema: {
                    type: "object";
                    properties: {
                      tokenType: {
                        type: string;
                      };
                      idToken: {
                        type: string;
                      };
                      accessToken: {
                        type: string;
                      };
                      refreshToken: {
                        type: string;
                      };
                      accessTokenExpiresAt: {
                        type: string;
                        format: string;
                      };
                      refreshTokenExpiresAt: {
                        type: string;
                        format: string;
                      };
                    };
                  };
                };
              };
            };
            400: {
              description: string;
            };
          };
        };
      };
    }, {
      accessToken: string | undefined;
      refreshToken: string | undefined;
      accessTokenExpiresAt: Date | undefined;
      refreshTokenExpiresAt: Date | undefined;
      scope: string | null | undefined;
      idToken: string | null | undefined;
      providerId: string;
      accountId: string;
    }>;
    readonly getAccessToken: better_call110.StrictEndpoint<"/get-access-token", {
      method: "POST";
      body: zod80.ZodObject<{
        providerId: zod80.ZodString;
        accountId: zod80.ZodOptional<zod80.ZodString>;
        userId: zod80.ZodOptional<zod80.ZodString>;
      }, zod_v4_core15.$strip>;
      metadata: {
        openapi: {
          description: string;
          responses: {
            200: {
              description: string;
              content: {
                "application/json": {
                  schema: {
                    type: "object";
                    properties: {
                      tokenType: {
                        type: string;
                      };
                      idToken: {
                        type: string;
                      };
                      accessToken: {
                        type: string;
                      };
                      refreshToken: {
                        type: string;
                      };
                      accessTokenExpiresAt: {
                        type: string;
                        format: string;
                      };
                      refreshTokenExpiresAt: {
                        type: string;
                        format: string;
                      };
                    };
                  };
                };
              };
            };
            400: {
              description: string;
            };
          };
        };
      };
    }, {
      accessToken: string;
      accessTokenExpiresAt: Date | undefined;
      scopes: string[];
      idToken: string | undefined;
    }>;
    readonly accountInfo: better_call110.StrictEndpoint<"/account-info", {
      method: "GET";
      use: ((inputContext: better_call110.MiddlewareInputContext<better_call110.MiddlewareOptions>) => Promise<{
        session: {
          session: Record<string, any> & {
            id: string;
            createdAt: Date;
            updatedAt: Date;
            userId: string;
            expiresAt: Date;
            token: string;
            ipAddress?: string | null | undefined;
            userAgent?: string | null | undefined;
          };
          user: Record<string, any> & {
            id: string;
            createdAt: Date;
            updatedAt: Date;
            email: string;
            emailVerified: boolean;
            name: string;
            image?: string | null | undefined;
          };
        };
      }>)[];
      metadata: {
        openapi: {
          description: string;
          responses: {
            "200": {
              description: string;
              content: {
                "application/json": {
                  schema: {
                    type: "object";
                    properties: {
                      user: {
                        type: string;
                        properties: {
                          id: {
                            type: string;
                          };
                          name: {
                            type: string;
                          };
                          email: {
                            type: string;
                          };
                          image: {
                            type: string;
                          };
                          emailVerified: {
                            type: string;
                          };
                        };
                        required: string[];
                      };
                      data: {
                        type: string;
                        properties: {};
                        additionalProperties: boolean;
                      };
                    };
                    required: string[];
                    additionalProperties: boolean;
                  };
                };
              };
            };
          };
        };
      };
      query: zod80.ZodOptional<zod80.ZodObject<{
        accountId: zod80.ZodOptional<zod80.ZodString>;
      }, zod_v4_core15.$strip>>;
    }, {
      user: _better_auth_core_oauth20.OAuth2UserInfo;
      data: Record<string, any>;
    } | null>;
  } & UnionToIntersection<Option["plugins"] extends (infer T)[] ? T extends BetterAuthPlugin ? T extends {
    endpoints: infer E;
  } ? E : {} : {} : {}>;
};
//#endregion
export { APIError, type AuthEndpoint, type AuthMiddleware, accountInfo, callbackOAuth, changeEmail, changePassword, checkEndpointConflicts, createAuthEndpoint, createAuthMiddleware, createEmailVerificationToken, deleteUser, deleteUserCallback, error, freshSessionMiddleware, getAccessToken, getEndpoints, getIp, getOAuthState, getSession, getSessionFromCtx, linkSocialAccount, listSessions, listUserAccounts, ok, optionsMiddleware, originCheck, originCheckMiddleware, refreshToken, requestOnlySessionMiddleware, requestPasswordReset, requestPasswordResetCallback, resetPassword, revokeOtherSessions, revokeSession, revokeSessions, router, sendVerificationEmail, sendVerificationEmailFn, sensitiveSessionMiddleware, sessionMiddleware, setPassword, signInEmail, signInSocial, signOut, signUpEmail, unlinkAccount, updateUser, verifyEmail };
//# sourceMappingURL=index.d.mts.map