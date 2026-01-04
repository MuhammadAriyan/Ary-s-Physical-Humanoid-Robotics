import { auth } from "../../src/auth.js";
import { toNodeHandler } from "better-auth/node";

// Disallow body parsing, we will parse it manually
export const config = {
  api: {
    bodyParser: false
  }
};

export default toNodeHandler(auth);