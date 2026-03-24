const fs = require('fs');
const path = 'tools/native_gui/dope_gui_cartridges.json';
let s = fs.readFileSync(path,'utf8');
// Remove trailing commas before closing array or object
s = s.replace(/,\s*([\]}])/g, '$1');
fs.writeFileSync(path,s,'utf8');
console.log('Fixed');
