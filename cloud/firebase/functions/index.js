const { onRequest } = require("firebase-functions/v2/https");
const { logger } = require("firebase-functions");
const { initializeApp } = require("firebase-admin/app");
const { getFirestore } = require("firebase-admin/firestore");

let config = null;
if (process.env.FUNCTIONS_EMULATOR) {
  config = {
    cors: ["http://127.0.0.1:5000"],
    region: "asia-southeast1",
  };
  console.log("Running in emulator mode");
} else {
  config = {
    cors: ["https://example.com"],
    region: "asia-southeast1",
  };
  console.log("Running in production mode");
}

const webRequestConfig = Object.freeze(config);
initializeApp();

exports.addmessage = onRequest(webRequestConfig, async (req, res) => {
  if (req.method !== "POST") {
    res.status(405).send("Method Not Allowed");
    return;
  }
  const jsonBody = req.body;
  const deviceId = jsonBody.device;
  const newDocument = await getFirestore().collection("devices").doc(deviceId);
  newDocument.collection("logs").add(jsonBody);

  res.json({ result: `Message with ID: ${newDocument.id} added.` });
});
