const $ = (selector) => document.querySelector(selector);
const $$ = (selector) => [...document.querySelectorAll(selector)];

const state = {
  intent: null,
  confirmed: false,
  program: "",
  programId: null,
  runId: null,
  running: false,
  round: 0,
  history: [],
  objects: {},
  recorder: null,
  recordStream: null,
  recordBlob: null,
  cameraStream: null,
  uploadUrl: null,
  pendingPatch: null,
  pendingCallRow: null,
  pendingCallText: "",
  eventSource: null,
  sidebarOpen: true,
};

function escapeHtml(value) {
  return String(value).replace(/[&<>"']/g, (char) => ({"&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;", "'": "&#39;"})[char]);
}

async function request(path, options = {}) {
  const response = await fetch(path, options);
  let body = {};
  try { body = await response.json(); } catch (_) { body = {}; }
  if (!response.ok) throw new Error(body.error || `${response.status} ${response.statusText}`);
  return body;
}

async function jsonRequest(path, body) {
  return request(path, {
    method: "POST",
    headers: {"Content-Type": "application/json"},
    body: JSON.stringify(body),
  });
}

function errorAt(selector, error = "") {
  const target = $(selector);
  target.textContent = error ? String(error.message || error) : "";
  if (error) {
    target.tabIndex = -1;
    target.focus();
  }
}

async function busy(button, label, work) {
  const old = button.textContent;
  button.disabled = true;
  button.textContent = label;
  try { return await work(); }
  finally { button.textContent = old; updateButtons(); }
}

function setStage(name, status = "active", note = "") {
  void name;
  void status;
  void note;
}

function setSidebar(open, {focus = false} = {}) {
  const sidebar = $("#sidebar");
  const toggle = $("#sidebar-toggle");
  state.sidebarOpen = open;
  sidebar.classList.toggle("open", open);
  sidebar.inert = !open;
  sidebar.setAttribute("aria-hidden", String(!open));
  document.body.classList.toggle("sidebar-closed", !open);
  toggle.setAttribute("aria-expanded", String(open));
  toggle.setAttribute("aria-label", open ? "Close sidebar" : "Open sidebar");
  toggle.querySelector("span").textContent = open ? "›" : "‹";
  if (!open && sidebar.contains(document.activeElement)) toggle.focus();
  else if (open && focus) requestAnimationFrame(() => sidebar.focus());
}

function revealWaiting() {
  const wasClosed = !state.sidebarOpen;
  setSidebar(true);
  requestAnimationFrame(() => {
    const block = $("#waiting-block");
    block.scrollIntoView({behavior: "smooth", block: "center"});
    if (wasClosed) block.focus({preventScroll: true});
  });
}

function renderViewRuntime(text = "", status = "") {
  const mark = status === "ok" ? '<span class="ok">✓</span>' : status === "fail" ? '<span class="fail">✗</span>' : "";
  $("#view-runtime").innerHTML = `${escapeHtml(text)}${mark ? ` ${mark}` : ""}`;
}

function renderViewFinal(ok, reason) {
  $("#view-runtime").innerHTML = ok
    ? 'Passed <span class="ok">✓</span>'
    : `Failed <span class="fail">✗</span> — ${escapeHtml(reason)}`;
}

function updateButtons() {
  const canRun = state.confirmed && state.programId && !state.running;
  const runHint = state.running ? "Run in progress" : !state.confirmed ? "Confirm a task first" : !state.programId || !state.program ? "Write the program first" : "Play";
  $("#run-btn").disabled = !canRun;
  $("#run-btn").title = runHint;
  $("#play-control").title = runHint;
  $("#rewrite-btn").disabled = !(state.intent && !state.running);
  $("#stop-btn").disabled = !state.running;
  $("#confirm-btn").disabled = !state.intent || state.running;
  $("#clarify-btn").disabled = !state.intent || state.running;
}

function invalidateProgram() {
  state.confirmed = false;
  state.program = "";
  state.programId = null;
  renderProgram("");
  $("#program-section").hidden = true;
  $("#program-meta").textContent = "";
  updateButtons();
}

// The rules exactly as the robot will enforce them (from the intent JSON, never the model's paraphrase).
function phrase(filter, {target = false} = {}) {
  const f = filter || {};
  const words = [f.size, f.color].filter(Boolean);
  const noun = f.category || (target ? "target" : "object");
  if (target) return `the ${[...words, noun].join(" ")}`;
  return words.length ? `${words.join(" ")} ${noun}s` : `every ${noun}`;
}

function readableIntent(intent) {
  return (intent.rules || []).map((rule) => {
    const subject = phrase(rule.select);
    const relation = rule.relation === "on" ? "onto" : "into";
    const text = `${subject.charAt(0).toUpperCase()}${subject.slice(1)} → ${relation} ${phrase(rule.target, {target: true})}`;
    return `<p>${escapeHtml(text)}</p>`;
  }).join("") + (intent.others === "stay" ? '<p class="seen">Everything else stays where it is.</p>' : "");
}

function showReading(data, correction = "") {
  state.intent = data.intent;
  state.round = data.round || state.round + 1;
  state.confirmed = false;
  $("#understood-section").hidden = false;
  const fromClip = state.fromClip;
  $("#understood-heading").textContent = fromClip ? "Understood" : "Rules";
  $("#seen-text").textContent = fromClip ? data.seen : "";
  $("#seen-text").hidden = !fromClip;
  $("#round-label").textContent = `Round ${state.round}`;
  $("#intent-readable").innerHTML = readableIntent(data.intent);
  $("#intent-json").value = JSON.stringify(data.intent, null, 2);
  $("#intent-json").hidden = true;
  $("#intent-readable").hidden = false;
  $("#edit-intent-btn").textContent = "Edit JSON";
  const note = data.note || "";
  $("#note-display").hidden = !note;
  $("#note-display").textContent = note;
  if (data.frames || !fromClip) $("#read-frames").innerHTML = (data.frames || []).map((src, index) => `<img src="${src}" alt="Sampled clip frame ${index + 1}" width="64" height="44">`).join("");
  if (correction) state.history.push(`round ${state.round}: ${correction}`);
  $("#intent-history").innerHTML = state.history.map((item) => `<div>${escapeHtml(item)}</div>`).join("");
  invalidateProgram();
  setStage("confirm", "active", "Check the reading before writing");
}

function syntaxLine(line) {
  const marker = line.indexOf("#");
  if (marker < 0) return escapeHtml(line) || " ";
  return `${escapeHtml(line.slice(0, marker))}<span class="tok-comment">${escapeHtml(line.slice(marker))}</span>`;
}

function renderProgram(source, currentLine = null) {
  state.program = source || "";
  if (!source) {
    $("#code-window").innerHTML = "";
    return;
  }
  $("#program-section").hidden = false;
  $("#code-window").innerHTML = source.replace(/\n$/, "").split("\n").map((line, index) => (
    `<div class="code-line ${currentLine === index + 1 ? "current" : ""}" data-line="${index + 1}"><span class="line-no">${index + 1}</span><span>${syntaxLine(line)}</span></div>`
  )).join("");
  if (currentLine) $(".code-line.current")?.scrollIntoView({block: "nearest"});
}

function addLog(kind, text, result = "", extraClass = "") {
  const log = $("#call-log");
  const row = document.createElement("div");
  row.className = `log-row ${extraClass}`;
  if (extraClass.includes("full")) {
    row.innerHTML = `<strong>${escapeHtml(kind)}</strong> · ${escapeHtml(text)}`;
  } else {
    row.innerHTML = `<span>${escapeHtml(text)}</span><span class="result ${result === "✓" ? "ok" : result ? "fail" : ""}">${escapeHtml(result)}</span>`;
  }
  log.append(row);
  log.scrollTop = log.scrollHeight;
  return row;
}

function lineDiff(oldSource, newSource) {
  const oldLines = oldSource.replace(/\n$/, "").split("\n");
  const newLines = newSource.replace(/\n$/, "").split("\n");
  const rows = Array.from({length: oldLines.length + 1}, () => Array(newLines.length + 1).fill(0));
  for (let i = oldLines.length - 1; i >= 0; i -= 1) {
    for (let j = newLines.length - 1; j >= 0; j -= 1) rows[i][j] = oldLines[i] === newLines[j] ? rows[i + 1][j + 1] + 1 : Math.max(rows[i + 1][j], rows[i][j + 1]);
  }
  const output = [];
  let i = 0;
  let j = 0;
  while (i < oldLines.length || j < newLines.length) {
    if (i < oldLines.length && j < newLines.length && oldLines[i] === newLines[j]) { output.push([" ", oldLines[i]]); i += 1; j += 1; }
    else if (j < newLines.length && (i === oldLines.length || rows[i][j + 1] >= rows[i + 1][j])) { output.push(["+", newLines[j]]); j += 1; }
    else { output.push(["−", oldLines[i]]); i += 1; }
  }
  return output;
}

function showApproval(data) {
  state.pendingPatch = data.patch;
  setStage("run", "active", "Waiting for you");
  $("#program-section").hidden = false;
  $("#waiting-block").hidden = false;
  $("#waiting-label").hidden = false;
  $("#waiting-content").innerHTML = `<p class="waiting-reason">${escapeHtml(data.reason || "Patch approval requested")}</p><div class="diff">` + lineDiff(state.program, data.patch).map(([mark, line]) => (
    `<div class="diff-line ${mark === "+" ? "add" : mark === "−" ? "remove" : ""}">${mark} ${escapeHtml(line)}</div>`
  )).join("") + `</div><div class="waiting-form"><label class="sr-only" for="decline-note">Optional note</label><input id="decline-note" name="decline-note" autocomplete="off" placeholder="Optional note…"><button class="outline-button" type="button" id="decline-patch">Decline</button><button class="outline-button" type="button" id="approve-patch">Approve</button></div>`;
  const submit = async (approved) => {
    try {
      await jsonRequest("/api/approve", {run_id: data.run_id, question_id: data.question_id, approved, note: $("#decline-note").value});
      clearWaiting();
      if (!approved) state.pendingPatch = null;
      addLog("PATCH", approved ? "Approved by operator" : "Declined by operator", "", "full");
    } catch (error) { errorAt("#program-error", error); }
  };
  $("#approve-patch").addEventListener("click", () => submit(true));
  $("#decline-patch").addEventListener("click", () => submit(false));
  revealWaiting();
}

function clearWaiting() {
  $("#waiting-block").hidden = true;
  $("#waiting-content").innerHTML = "";
  $("#waiting-label").hidden = true;
}

function showQuestion(data) {
  $("#program-section").hidden = false;
  $("#waiting-block").hidden = false;
  $("#waiting-label").hidden = false;
  $("#waiting-content").innerHTML = `<p class="waiting-reason">${escapeHtml(data.text)}</p><div class="waiting-form"><label class="sr-only" for="question-answer">Answer</label><input id="question-answer" name="question-answer" autocomplete="off" placeholder="Your answer…"><button class="outline-button" type="button" id="send-answer">Send</button></div>`;
  $("#send-answer").addEventListener("click", async () => {
    try {
      const text = $("#question-answer").value;
      await jsonRequest("/api/answer", {run_id: data.run_id, question_id: data.question_id, text});
      clearWaiting();
      addLog("QUESTION", `${data.text} — ${text}`, "", "full");
    } catch (error) { errorAt("#program-error", error); }
  });
  revealWaiting();
}

function handleRuntime(event) {
  const data = JSON.parse(event.data);
  if (data.kind === "line") renderProgram(state.program, data.line);
  else if (data.kind === "call") {
    state.pendingCallText = `${data.fn}(${(data.args || []).map((arg) => typeof arg === "string" ? arg : JSON.stringify(arg)).join(", ")})`;
    state.pendingCallRow = addLog("CALL", state.pendingCallText);
    renderViewRuntime(state.pendingCallText);
  }
  else if (data.kind === "result") {
    const value = data.ok ? `✓${data.message ? ` ${data.message}` : ""}` : `✗ ${data.message || "failed"}`;
    if (state.pendingCallRow) {
      const result = state.pendingCallRow.querySelector(".result");
      result.className = `result ${data.ok ? "ok" : "fail"}`;
      result.textContent = value;
      state.pendingCallRow = null;
    } else addLog("RESULT", data.fn, value);
    renderViewRuntime(state.pendingCallText || data.fn || "Call", data.ok ? "ok" : "fail");
  }
  else if (["handoff", "failure", "report", "decision", "dry_run", "check"].includes(data.kind)) addLog(data.kind.toUpperCase(), data.message || data.text || data.reason || (data.ok === false ? data.error : "checked"), "", "full");
  else if (data.kind === "patch" && data.accepted && state.pendingPatch) { renderProgram(state.pendingPatch); state.pendingPatch = null; }
  else if (data.kind === "done" && !data.ok) addLog("DONE", data.error || "Run failed", "", "full");
}

function renderFinal(data) {
  state.running = false;
  state.runId = null;
  state.pendingPatch = null;
  clearWaiting();
  $("#run-state").textContent = data.ok ? "Complete" : "Stopped";
  $("#verdict").className = `verdict ${data.ok ? "pass" : "fail"}`;
  const reason = data.error || data.rules?.summary || "Run did not complete";
  $("#verdict").textContent = data.ok ? "Passed ✓" : `Failed ✗ — ${reason}`;
  renderViewFinal(data.ok, reason);
  state.pendingCallText = "";
  if (!data.rules) {
    $("#result-body").innerHTML = "";
  } else {
    const rows = Object.entries(data.rules.per_object || {}).map(([id, status]) => {
      const object = state.objects[id] || {};
      const ok = status === "ok";
      return `<div class="check-row"><span class="check-mark ${ok ? "pass" : "fail"}">${ok ? "✓" : "✗"}</span><span class="check-object">${escapeHtml(id)}<small>${escapeHtml([object.color, object.category].filter(Boolean).join(" · "))}</small></span><span class="check-status">${escapeHtml(status)}</span></div>`;
    }).join("");
    $("#result-body").innerHTML = rows;
  }
  setStage("check", data.ok ? "done" : "failed", data.error || (data.ok ? "All checks complete" : "Run failed"));
  updateButtons();
}

function connectEvents() {
  state.eventSource?.close();
  const source = new EventSource("/api/events");
  state.eventSource = source;
  source.addEventListener("runtime", handleRuntime);
  source.addEventListener("stage", (event) => {
    const data = JSON.parse(event.data);
    setStage(data.name, data.state === "failed" ? "failed" : "active", data.detail || "");
  });
  source.addEventListener("question", (event) => {
    const data = JSON.parse(event.data);
    setStage("run", "active", "Waiting for you");
    showQuestion(data);
  });
  source.addEventListener("approval", (event) => showApproval(JSON.parse(event.data)));
  source.addEventListener("final", (event) => { renderFinal(JSON.parse(event.data)); source.close(); });
}

async function loadScenes() {
  try {
    const scenes = await request("/api/scenes");
    $("#scene-select").innerHTML = scenes.map((scene) => `<option value="${escapeHtml(scene.id)}" data-request="${escapeHtml(scene.request)}">${escapeHtml(scene.label)}</option>`).join("");
    $("#task-text").value = scenes[0].request;
    await loadScene();
  } catch (error) { errorAt("#task-error", error); }
}

async function loadScene(button = $("#load-scene-btn")) {
  errorAt("#task-error");
  await busy(button, "Loading…", async () => {
    const data = await jsonRequest("/api/scene", {id: $("#scene-select").value});
    state.objects = Object.fromEntries(data.objects.map((object) => [object.id, object]));
    state.intent = null;
    state.history = [];
    state.confirmed = false;
    $("#understood-section").hidden = true;
    invalidateProgram();
    setStage("read", "active", "Scene ready");
  });
}

async function readText() {
  errorAt("#task-error");
  const button = $("#send-text-btn");
  try {
    const data = await busy(button, "Reading…", () => jsonRequest("/api/read", {text: $("#task-text").value}));
    state.history = [];
    state.fromClip = false;
    showReading(data);
  } catch (error) { errorAt("#task-error", error); setStage("read", "failed", error.message); }
}

async function readClip(blob, note, button) {
  errorAt("#task-error");
  try {
    const type = blob.type || "video/webm";
    const data = await busy(button, "Reading…", () => request(`/api/read_clip?note=${encodeURIComponent(note)}`, {method: "POST", headers: {"Content-Type": type}, body: blob}));
    state.history = [];
    state.fromClip = true;
    showReading(data);
  } catch (error) { errorAt("#task-error", error); setStage("read", "failed", error.message); }
}

async function writeProgram(button) {
  errorAt("#intent-error");
  let intent;
  try { intent = JSON.parse($("#intent-json").value); }
  catch (error) { errorAt("#intent-error", new Error(`Invalid JSON: ${error.message}`)); return; }
  state.confirmed = false;
  state.programId = null;
  updateButtons();
  try {
    setStage("write", "active", "Writing a checked program");
    const data = await busy(button, "Writing…", () => jsonRequest("/api/write", {intent, confirm: true}));
    state.intent = intent;
    state.confirmed = true;
    state.programId = data.program_id;
    renderProgram(data.program);
    $("#program-meta").textContent = `${data.attempts} attempt${data.attempts === 1 ? "" : "s"}`;
    setStage("run", "active", "Program ready — press Run");
  } catch (error) { errorAt("#intent-error", error); setStage("write", "failed", error.message); }
  finally { updateButtons(); }
}

$$('[data-input]').forEach((button) => button.addEventListener("click", async () => {
  const target = `#clip-${button.dataset.input}`;
  const opening = $(target).hidden;
  $$(".clip-input").forEach((item) => { item.hidden = true; });
  $$("[data-input]").forEach((item) => item.classList.remove("active"));
  $(target).hidden = !opening;
  button.classList.toggle("active", opening);
  if (opening && button.dataset.input === "record" && !state.recordStream) {
    try { state.recordStream = await navigator.mediaDevices.getUserMedia({video: true, audio: false}); $("#record-preview").srcObject = state.recordStream; await $("#record-preview").play(); }
    catch (error) { errorAt("#task-error", error); }
  }
}));

$("#scene-select").addEventListener("change", () => {
  const option = $("#scene-select").selectedOptions[0];
  if (option?.dataset.request) $("#task-text").value = option.dataset.request;
});
$("#load-scene-btn").addEventListener("click", () => loadScene($("#load-scene-btn")).catch((error) => errorAt("#task-error", error)));
$("#reset-btn").addEventListener("click", () => loadScene($("#reset-btn")).catch((error) => errorAt("#task-error", error)));
$("#send-text-btn").addEventListener("click", readText);
$("#task-text").addEventListener("keydown", (event) => { if (event.ctrlKey && event.key === "Enter") readText(); });

$("#record-btn").addEventListener("click", async () => {
  if (!state.recordStream) return;
  const chunks = [];
  state.recorder = new MediaRecorder(state.recordStream, {mimeType: "video/webm"});
  state.recorder.addEventListener("dataavailable", (event) => { if (event.data.size) chunks.push(event.data); });
  state.recorder.addEventListener("stop", () => {
    state.recordBlob = new Blob(chunks, {type: "video/webm"});
    $("#record-preview").srcObject = null;
    $("#record-preview").src = URL.createObjectURL(state.recordBlob);
    $("#record-preview").controls = true;
    $("#send-record-btn").disabled = false;
  });
  state.recorder.start();
  $("#record-btn").disabled = true;
  $("#record-stop-btn").disabled = false;
});
$("#record-stop-btn").addEventListener("click", () => { state.recorder?.stop(); $("#record-stop-btn").disabled = true; });
$("#send-record-btn").addEventListener("click", () => readClip(state.recordBlob, $("#record-note").value, $("#send-record-btn")));

$("#clip-file").addEventListener("change", () => {
  const file = $("#clip-file").files[0];
  $("#file-name").textContent = file?.name || "";
  $("#send-upload-btn").disabled = !file;
  if (state.uploadUrl) URL.revokeObjectURL(state.uploadUrl);
  state.uploadUrl = file ? URL.createObjectURL(file) : null;
  $("#upload-preview").hidden = !file;
  $("#upload-preview").src = state.uploadUrl || "";
});
$("#send-upload-btn").addEventListener("click", () => readClip($("#clip-file").files[0], $("#upload-note").value, $("#send-upload-btn")));

$("#clarify-btn").addEventListener("click", async () => {
  const correction = $("#clarify-text").value.trim();
  if (!correction) return;
  errorAt("#intent-error");
  try {
    const data = await busy($("#clarify-btn"), "Reading…", () => jsonRequest("/api/clarify", {text: correction}));
    showReading(data, correction);
    $("#clarify-text").value = "";
  } catch (error) { errorAt("#intent-error", error); }
});
$("#edit-intent-btn").addEventListener("click", () => {
  const raw = $("#intent-json");
  raw.hidden = !raw.hidden;
  $("#intent-readable").hidden = !raw.hidden;
  $("#edit-intent-btn").textContent = raw.hidden ? "Edit JSON" : "Readable view";
  invalidateProgram();
});
$("#confirm-btn").addEventListener("click", () => writeProgram($("#confirm-btn")));
$("#rewrite-btn").addEventListener("click", () => writeProgram($("#rewrite-btn")));

$("#run-btn").addEventListener("click", async () => {
  errorAt("#program-error");
  try {
    const data = await jsonRequest("/api/run", {program_id: state.programId});
    state.runId = data.run_id;
    state.running = true;
    state.pendingCallRow = null;
    state.pendingCallText = "";
    $("#run-state").textContent = "Running";
    $("#call-log").innerHTML = "";
    $("#result-body").innerHTML = "";
    $("#verdict").className = "verdict";
    $("#verdict").textContent = "";
    renderViewRuntime();
    setStage("run", "active", "Executing the approved program");
    updateButtons();
    connectEvents();
  } catch (error) { errorAt("#program-error", error); }
});

$("#stop-btn").addEventListener("click", async () => {
  try { await busy($("#stop-btn"), "Stopping…", () => jsonRequest("/api/stop", {})); }
  catch (error) { errorAt("#program-error", error); }
});

$$('[data-source]').forEach((button) => button.addEventListener("click", async () => {
  $$('[data-source]').forEach((item) => item.classList.toggle("active", item === button));
  const camera = button.dataset.source === "camera";
  $("#sim-view").hidden = camera;
  $("#camera-view").hidden = !camera;
  if (camera && !state.cameraStream) {
    try { state.cameraStream = await navigator.mediaDevices.getUserMedia({video: true, audio: false}); $("#camera-view").srcObject = state.cameraStream; }
    catch (error) { errorAt("#view-error", error); }
  }
}));

async function pollStatus() {
  try {
    const data = await request("/api/status");
    const problems = [];
    if (data.robot.status === "error") problems.push("Robot offline");
    if (data.model.status === "error") problems.push("Model unreachable");
    $("#system-status").textContent = problems.join(" · ");
    $("#system-status").title = [data.robot.message, data.model.message].filter(Boolean).join(" · ");
    const fresh = data.frames !== null && data.frames <= 2;
    $("#live-dot").className = `feed-dot ${fresh ? "fresh" : ""}`;
    $("#live-dot").setAttribute("aria-label", fresh ? "Feed current" : "Feed stale");
    const waiting = data.run === "waiting";
    $("#waiting-label").hidden = !waiting;
    if (waiting && !state.sidebarOpen && !$("#waiting-block").hidden) revealWaiting();
    $("#run-state").textContent = data.run === "idle" ? "" : data.run[0].toUpperCase() + data.run.slice(1);
  } catch (_) { $("#system-status").textContent = "Status unavailable"; }
}

$("#sidebar-toggle").addEventListener("click", () => setSidebar(!state.sidebarOpen, {focus: !state.sidebarOpen}));
$(".skip-link").addEventListener("click", () => setSidebar(true, {focus: true}));
// ?panel: controls only, for docking beside the real Isaac window (native camera controls, full render quality).
const PANEL_ONLY = new URLSearchParams(location.search).has("panel");
if (PANEL_ONLY) {
  document.body.classList.add("panel-only");
  $("#sim-view").removeAttribute("src");
}

document.addEventListener("keydown", (event) => {
  if (PANEL_ONLY) return;
  const target = event.target;
  const typing = target instanceof HTMLElement && (target.matches("input, textarea, select") || target.isContentEditable);
  if (event.key === "Escape" && state.sidebarOpen) {
    event.preventDefault();
    setSidebar(false);
  } else if (!typing && event.key.toLowerCase() === "s" && !event.ctrlKey && !event.metaKey && !event.altKey) {
    event.preventDefault();
    setSidebar(!state.sidebarOpen, {focus: !state.sidebarOpen});
  }
});

setInterval(pollStatus, 2000);
setSidebar(true);
loadScenes();
pollStatus();
updateButtons();
