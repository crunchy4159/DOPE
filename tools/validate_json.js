const fs = require('fs');
const path = 'tools/native_gui/dope_gui_cartridges.json';
try {
  const s = fs.readFileSync(path,'utf8');
  JSON.parse(s);
  console.log('OK');
} catch (e) {
  console.error('ERROR', e && e.stack ? e.stack : (e && e.message ? e.message : e));
  process.exit(1);
}
