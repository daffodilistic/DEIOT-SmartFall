const { onRequest } = require("firebase-functions/v2/https");
const { logger } = require("firebase-functions");
const { initializeApp } = require("firebase-admin/app");
const { getFirestore } = require("firebase-admin/firestore");

const webRequestConfig = {
    cors: ["https://example.com"],
    region: 'asia-southeast1'
}

initializeApp();

exports.addmessage = onRequest(webRequestConfig, async (req, res) => {
    if (req.method !== "POST") {
        res.status(405).send("Method Not Allowed");
        return;
    }
    const jsonBody = req.body;
    const writeResult = await getFirestore()
        .collection("logs")
        .add(jsonBody);
    res.json({ result: `Message with ID: ${writeResult.id} added.` });
});
