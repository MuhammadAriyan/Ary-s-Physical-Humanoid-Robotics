import { constantTimeEqual } from "./buffer.mjs";
import { signJWT, symmetricDecodeJWT, symmetricEncodeJWT, verifyJWT } from "./jwt.mjs";
import { hashPassword, verifyPassword } from "./password.mjs";
import { generateRandomString } from "./random.mjs";

//#region src/crypto/index.d.ts
type SymmetricEncryptOptions = {
  key: string;
  data: string;
};
declare const symmetricEncrypt: ({
  key,
  data
}: SymmetricEncryptOptions) => Promise<string>;
type SymmetricDecryptOptions = {
  key: string;
  data: string;
};
declare const symmetricDecrypt: ({
  key,
  data
}: SymmetricDecryptOptions) => Promise<string>;
//#endregion
export { SymmetricDecryptOptions, SymmetricEncryptOptions, constantTimeEqual, generateRandomString, hashPassword, signJWT, symmetricDecodeJWT, symmetricDecrypt, symmetricEncodeJWT, symmetricEncrypt, verifyJWT, verifyPassword };
//# sourceMappingURL=index.d.mts.map