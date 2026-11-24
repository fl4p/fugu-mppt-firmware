// Warns when a page mentions a topic several times but never links the page that covers it.
// Warning only, never fails the build: the term list is a heuristic.
import path from 'node:path';
import {fileURLToPath} from 'node:url';
import {visit} from 'unist-util-visit';

const docsDir = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '../../docs');
const MIN_MENTIONS = 3;

// target page (relative to docs/) -> mention pattern, or [pattern, minimum mentions]. Keep patterns specific: a generic word
// ("profile", "telemetry", "dead time") matches pages that use it in another sense.
const TOPICS = {
  'internals/dcm-ringing.mdx': /\bDCM ring|switch-node ring|\bC_sw\b|snubber/gi,
  'internals/diode-emulation.md': /diode emulation|\bZCD\b|rectCtrlRatio|computeDCM/gi,
  'internals/rect-cal.md': /\brect-cal\b|rectifier timing calibration/gi,
  'lab/coil-inductance.md': /measure-coil|measure_coil|coil inductance/gi,
  'lab/switch-node-capacitance.md': /switch-node capacitance|node capacitance/gi,
  'internals/mppt-tracker.md': /global sweep|\bP&O\b|scan_near|peak scan/gi,
  'internals/signal-filters.md': /\bnotch\b|moving median|comb filter/gi,
  'internals/pwm-drivers.md': [/\bMCPWM\b|\bLEDC\b/gi, 6],
  'internals/control-loop.md': /\bloopRT\b|PD controller|VinCTRL|VoutCTRL|IoutCTRL|PowerCTRL/gi,
  'internals/lfp-longevity.md': /calendar ag|LFP longevity|cycle life/gi,
  'guide/charging/termination.md': /\bcv_eoc\b|\bcv_float\b|charge termination/gi,
  'guide/charging/bms-integration.md': /BMS (data|topic|feed|integration)|cell voltage/gi,
  'development/sync/beacon-sync.md': /\bbsync\b/gi,
  'development/sync/wired-sync.md': /\bwsync\b|wired sync/gi,
  'development/simulation.md': /\bvconv\b|mppt-sim|virtual converter/gi,
  'development/debugging/real-time-latency.md': /loop latency|flash auto-suspend/gi,
  'development/debugging/peek.md': /\bpeek(-struct)?\b/gi,
  'development/debugging/rtcount.md': /\brtcount\b/gi,
  'development/debugging/profiling.md': /sprofiler|\brtprof\b|profiling/gi,
  'guide/updating/ota-ble.md': /\bota-ble\b|\bota_ble\b|OTA over BLE/gi,
  'guide/usb-drive.md': /USB_MSC|USB config drive/gi,
  'reference/config/extport.md': /VE\.Direct|\bextport\b/gi,
  'guide/telemetry/index.md': [/InfluxDB|Home Assistant/gi, 5],
  'reference/services.md': /ServiceManager/gi,
  'guide/hardware/internal-adc.md': /internal ADC|ADC_ESP32_Cont/gi,
  'reference/config-editor.md': /conf-editor|config editor/gi,
  'lab/power-loop.md': /power-loop/gi,
  'lab/automated-bench-tests.md': /run_e2e|e2e test/gi,
  'development/bootguard.md': /bootguard|crash-loop guard/gi,
  'reference/ble/gatt.md': /\bGATT\b|\bNUS\b/gi,
  'development/wokwi.md': /Wokwi/gi,
};
// A link to one of these pages also counts as a link to the target.
const ALSO = {
  'development/simulation.md': ['reference/config/vconv.md'],
  'development/sync/beacon-sync.md': ['development/sync/index.md', 'reference/config/bsync.md'],
  'development/sync/wired-sync.md': ['development/sync/index.md'],
  'guide/telemetry/index.md': ['reference/telemetry-fields.md', 'reference/config/tele.md'],
  'guide/charging/bms-integration.md': ['reference/config/mqtt.md'],
  'guide/updating/ota-ble.md': ['guide/updating/ble-ota-transports.md'],
};
// Pages that list every topic in passing (command and flag references) are exempt.
const EXEMPT = new Set(['reference/console.md']);

const norm = (p) => p.replace(/\\/g, '/');

export default function docsXlink() {
  return (tree, file) => {
    const rel = norm(path.relative(docsDir, file.path));
    if (rel.startsWith('..') || EXEMPT.has(rel)) return;
    const linked = new Set();
    let text = '';
    visit(tree, (node) => {
      if (node.type === 'link' && node.url && !/^[a-z]+:/i.test(node.url)) {
        const u = node.url.split('#')[0];
        if (u) linked.add(norm(path.relative(docsDir, path.resolve(path.dirname(file.path), u))));
      }
      if (node.type === 'text' || node.type === 'inlineCode') text += node.value + '\n';
    });
    const missing = [];
    for (const [target, spec] of Object.entries(TOPICS)) {
      const [re, min] = Array.isArray(spec) ? spec : [spec, MIN_MENTIONS];
      if (target === rel || linked.has(target) || (ALSO[target] || []).some((a) => a === rel || linked.has(a))) continue;
      const n = (text.match(re) || []).length;
      if (n >= min) missing.push(`${target} (${n}×)`);
    }
    if (missing.length) console.warn(`[docs-xlink] ${rel}: mentions without a link to ${missing.join(', ')}`);
  };
}
