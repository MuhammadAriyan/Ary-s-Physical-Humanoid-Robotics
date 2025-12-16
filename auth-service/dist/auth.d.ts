export declare const auth: import("better-auth").Auth<{
    database: import("pg").Pool;
    baseURL: string;
    secret: string | undefined;
    user: {
        modelName: string;
    };
    session: {
        modelName: string;
        expiresIn: number;
        updateAge: number;
        cookieCache: {
            enabled: true;
            maxAge: number;
        };
    };
    account: {
        modelName: string;
        accountLinking: {
            enabled: true;
            trustedProviders: "google"[];
        };
    };
    verification: {
        modelName: string;
    };
    emailAndPassword: {
        enabled: true;
        requireEmailVerification: false;
    };
    socialProviders: {
        google?: {
            clientId: string;
            clientSecret: string;
        } | undefined;
    };
    trustedOrigins: string[];
    advanced: {};
}>;
export type Auth = typeof auth;
//# sourceMappingURL=auth.d.ts.map