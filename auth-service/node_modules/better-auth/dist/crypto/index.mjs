import { constantTimeEqual } from "./buffer.mjs";
import { signJWT, symmetricDecodeJWT, symmetricEncodeJWT, verifyJWT } from "./jwt.mjs";
import { hashPassword, verifyPassword } from "./password.mjs";
import { generateRandomString } from "./random.mjs";
import { createHash } from "@better-auth/utils/hash";
import { xchacha20poly1305 } from "@noble/ciphers/chacha.js";
import { bytesToHex, hexToBytes, managedNonce, utf8ToBytes } from "@noble/ciphers/utils.js";

//#region src/crypto/index.ts
const symmetricEncrypt = async ({ key, data }) => {
	const keyAsBytes = await createHash("SHA-256").digest(key);
	const dataAsBytes = utf8ToBytes(data);
	return bytesToHex(managedNonce(xchacha20poly1305)(new Uint8Array(keyAsBytes)).encrypt(dataAsBytes));
};
const symmetricDecrypt = async ({ key, data }) => {
	const keyAsBytes = await createHash("SHA-256").digest(key);
	const dataAsBytes = hexToBytes(data);
	const chacha = managedNonce(xchacha20poly1305)(new Uint8Array(keyAsBytes));
	return new TextDecoder().decode(chacha.decrypt(dataAsBytes));
};

//#endregion
export { constantTimeEqual, generateRandomString, hashPassword, signJWT, symmetricDecodeJWT, symmetricDecrypt, symmetricEncodeJWT, symmetricEncrypt, verifyJWT, verifyPassword };
//# sourceMappingURL=index.mjs.map