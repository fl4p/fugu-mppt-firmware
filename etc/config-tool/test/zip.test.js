// Spec §1 — built-in ZIP reader (stored + deflate) and writer (stored).
const test = require('node:test');
const assert = require('node:assert/strict');
const { loadEditor } = require('./_setup');

test('crc32 matches the well-known value for "123456789"', async () => {
  const { window } = await loadEditor();
  const data = new TextEncoder().encode('123456789');
  // Canonical CRC-32 (ISO 3309) of "123456789" is 0xCBF43926
  assert.equal(window.crc32(data), 0xcbf43926);
});

test('writeZip + readZip round-trip preserves file names and bytes', async () => {
  const { window } = await loadEditor();
  const enc = new TextEncoder();
  const input = {
    'config/conf/board.conf'  : enc.encode('mcu=esp32s3\n'),
    'config/conf/limits.conf' : enc.encode('hv_max=80\nlv_max=60\n'),
    'config/raw.bin'          : new Uint8Array([0,1,2,3,4,5]),
  };
  const blob = window.writeZip(input);
  const buf  = await blob.arrayBuffer();
  const out  = await window.readZip(buf);

  assert.deepEqual(Object.keys(out).sort(), Object.keys(input).sort());
  for (const name of Object.keys(input)) {
    assert.deepEqual(Array.from(out[name]), Array.from(input[name]),
      'bytes for ' + name + ' should round-trip');
  }
});

test('readZip rejects non-zip input', async () => {
  const { window } = await loadEditor();
  await assert.rejects(window.readZip(new ArrayBuffer(16)), /not a zip/);
});

// §3 — an untouched config downloads as a dated backup; once edited it is "-edited".
test('download names an untouched config "<src>-backup-<date>.zip", "-edited" once changed', async () => {
  const { window } = await loadEditor();
  const enc = new TextEncoder();
  window.load({ 'config/conf/board.conf': enc.encode('mcu=esp32s3\npwm_freq=39000\n') }, 'fry');
  window._state.fromDevice = true; window._state.deviceName = 'fry';

  let dl = null;
  window.URL.createObjectURL = () => 'blob:stub';
  window.URL.revokeObjectURL = () => {};
  window.HTMLAnchorElement.prototype.click = function(){ dl = this.download; };

  // unedited → backup name with today's local date
  await window.download();
  const d = new Date();
  const today = `${d.getFullYear()}-${String(d.getMonth()+1).padStart(2,'0')}-${String(d.getDate()).padStart(2,'0')}`;
  assert.equal(dl, `fry-backup-${today}.zip`);

  // edit a field → "-edited"
  const r = [...window.document.querySelectorAll('#panes .row')].find(x => x._key === 'pwm_freq');
  r.querySelector('input').value = '40000';
  r.querySelector('input').dispatchEvent(new window.Event('input'));
  await window.download();
  assert.equal(dl, 'fry-edited.zip');
});
