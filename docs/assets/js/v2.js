function getFreeSpeedFromMotors() {
  let motor_count = {}
  let modules = ['frontright', 'frontleft', 'backright', 'backleft'];
  let highest_count = 0;
  let highest_motor = '';
  for (var module of modules) {
    let motor = $("#" + module + "angle_type-input").val();
    if (isNaN(motor_count[motor])) {
      motor_count[motor] = 0;
    }
    motor_count[motor] = motor_count[motor] + 1;
    if (motor_count[motor] > highest_count) {
      highest_count = motor_count[motor];
      highest_motor = motor;
    }
  }

  console.log(motor_count);
  switch (highest_motor) {
    case "neo":
      // $("#freespeedrpm-input").val(5676);
    case "falcon":
      // $("#freespeedrpm-input").val(6800);
  }
}

function updateAll() {
  // let jsons = ['pidfproperties', 'controllerproperties', 'physicalproperties', 'frontleft',
  //   'frontright', 'backleft', 'backright', 'swervedrive', 'controllerproperties']; // TODO: Commented out until all of the forms are built
  let jsons = ['swervedrive', 'physicalproperties', 'frontleft', 'frontright',
    'backleft', 'backright', 'controllerproperties', 'pidfproperties'];
  jsons.forEach((json) => {
    updateJSON(json);
  });

  getFreeSpeedFromMotors();
}

function updateJSON(name) {
  let form = $(`#${name}-json`);
  if (form.text() !== jsonify(name)) {
    form.text(jsonify(name));
  }
  // $(`#${name}-test`).text(jsonify(name));
}

function jsonify(name) {
  let data = {};
  let formdata = $(`#${name}-form`).serializeArray();
  formdata.forEach((val) => {
    if (val.name.includes('_')) {
      let structStart = val.name.indexOf('_')
      let struct = val.name.substring(0, structStart);
      let subStruct = null;
      if (data[struct] === undefined) {
        data[struct] = {};
      }
      let subStructStart = val.name.indexOf('_', structStart + 1);
      if (subStructStart !== -1) {
        subStruct = val.name.substring(structStart + 1, subStructStart)
        if (data[struct][subStruct] === undefined) {
          data[struct][subStruct] = {}
        }
      }
      if (subStruct == null) {
        const field = val.name.substring(val.name.lastIndexOf('_') + 1);
      const $input = $(`#${name}-form [name="${val.name}"]`);
      const isCanBusField = val.name.toLowerCase().includes('canbus');
      data[struct][field] = isCanBusField && $input.attr('data-is-null') === 'true' ? 
          null : 
          (isNumeric(val.value) ? parseFloat(val.value) : (val.value === "" ? null : val.value));
      } else {
        const field = val.name.substring(val.name.lastIndexOf('_') + 1);
        const $input = $(`#${name}-form [name="${val.name}"]`);
        const isCanBusField = val.name.toLowerCase().includes('canbus');
        data[struct][subStruct][field] = isCanBusField && $input.attr('data-is-null') === 'true' ? 
            null : 
            (isNumeric(val.value) ? parseFloat(val.value) : (val.value === "" ? null : val.value));
      }
    } else {
      data[val.name] = isNumeric(val.value) ?
          parseFloat(val.value) :
          (val.value === "" ? null : val.value);
    }
  });
  $(`#${name}-form input:checkbox`).each(function () { // Update checkboxes because unchecked checkboxes are not recognized by jQuery serializeArray
    if (this.name.includes('_')) {
      let struct = this.name.substring(0, this.name.indexOf('_'));
      if (data[struct] === undefined) {
        data[struct] = {};
      }
      data[struct][this.name.substring(
          this.name.indexOf('_') + 1)] = this.checked;
    } else {
      data[this.name] = this.checked;
    }
  });
  if (name == "swervedrive") {
    data['modules'] =
        [
          "frontleft.json",
          "frontright.json",
          "backleft.json",
          "backright.json"
        ]
  }
  return JSON.stringify(data, null, 2);
}

function isNumeric(str) {
  if (typeof str != "string") {
    return false
  } // we only process strings!
  return !isNaN(str) && // use type coercion to parse the _entirety_ of the string (`parseFloat` alone does not do this)...
      !isNaN(parseFloat(str)) // ...and ensure strings of whitespace fail
}

function copyText(name) {
  let text = $(`#${name}-json`).text();
  navigator.clipboard.writeText(text);
}

function getText(name) {
  let text = $(`#${name}-json`).text();
  return text;
}

//function from https://github.com/eligrey/FileSaver.js/issues/774
const saveAs = (blob, name) => {
  // Namespace is used to prevent conflict w/ Chrome Poper Blocker extension (Issue https://github.com/eligrey/FileSaver.js/issues/561)
  const a = document.createElementNS('http://www.w3.org/1999/xhtml', 'a')
  a.download = name
  a.rel = 'noopener'
  a.href = URL.createObjectURL(blob)

  setTimeout(() => URL.revokeObjectURL(a.href), 40 /* sec */ * 1000)
  setTimeout(() => a.click(), 0)
}

function zipDownload() {
  const zip = new JSZip();
  let swf = zip.folder("swerve")
  let cp = swf.file("controllerproperties.json",
      getText("controllerproperties"));
  let sd = swf.file("swervedrive.json", getText("swervedrive"));
  let mod = swf.folder("modules")
  let pp = mod.file("physicalproperties.json", getText("physicalproperties"));
  let fl = mod.file("frontleft.json", getText("frontleft"));
  let fr = mod.file("frontright.json", getText("frontright"));
  let bl = mod.file("backleft.json", getText("backleft"));
  let br = mod.file("backright.json", getText("backright"));
  let pidf = mod.file("pidfproperties.json", getText("pidfproperties"));

  zip.generateAsync({type: "blob"}).then(function (blob) {
    saveAs(blob, "YAGSL Config.zip")
  });
  console.log("Downloaded YAGSL Config zip");
}

function addZipImporter() {
  // If import button already exists, don't recreate; just ensure it's placed correctly
  let existingImport = document.getElementById('import-zip-button');
  let fileInput = document.getElementById('zip-file-input');

  // Create hidden file input if missing
  if (!fileInput) {
    fileInput = document.createElement('input');
    fileInput.type = 'file';
    fileInput.accept = '.zip,application/zip';
    fileInput.id = 'zip-file-input';
    fileInput.style.display = 'none';
    document.body.appendChild(fileInput);
  }
  // Attach change listener once
  if (!fileInput.__yagsl_listener_attached) {
    fileInput.addEventListener('change', (e) => {
      const f = e.target.files && e.target.files[0];
      if (f) {
        importZipFile(f);
      }
      fileInput.value = '';
    });
    fileInput.__yagsl_listener_attached = true;
  }

  // Helper to create a button only when needed
  function makeButton(id, className, text, inlineStyle) {
    const btn = document.createElement('button');
    btn.id = id;
    btn.type = 'button';
    btn.className = className;
    btn.textContent = text;
    if (inlineStyle) btn.style.cssText = inlineStyle;
    return btn;
  }

  if (!existingImport) {
    existingImport = makeButton('import-zip-button', 'btn btn-primary btn-lg mt-2 mb-3', 'Import ZIP', null);
    existingImport.addEventListener('click', () => fileInput.click());
  }

  // No test button: removed per user request

  // Place buttons next to existing download button if present
  const downloadContainer = document.getElementById('download-button');
  if (downloadContainer) {
    // Force layout with !important to ensure consistency
    downloadContainer.style.cssText = `
      display: flex !important;
      justify-content: center !important;
      align-items: center !important;
      gap: 10px !important;
      flex-wrap: nowrap !important;
      margin-bottom: 1rem !important;
    `;
    
    // Avoid re-appending if already inside
    if (!downloadContainer.contains(existingImport)) downloadContainer.appendChild(existingImport);
    if (!downloadContainer.contains(fileInput)) downloadContainer.appendChild(fileInput);
    
    // Find and move the Run Import Tests button if it exists (use the known id)
    const runImportButton = document.getElementById('run-import-tests');
    if (runImportButton && !downloadContainer.contains(runImportButton)) {
      downloadContainer.appendChild(runImportButton);
    }
    return;
  }

  // Fallback: create fixed container at top-right (used for embedded mode)
  let fixedButtons = document.getElementById('fixed-buttons');
  if (!fixedButtons) {
    fixedButtons = document.createElement('div');
    fixedButtons.id = 'fixed-buttons';
    fixedButtons.style.cssText = `
      position: fixed;
      top: 20px;
      right: 20px;
      z-index: 1000;
      display: flex;
      gap: 10px;
      flex-direction: column;
    `;
    document.body.appendChild(fixedButtons);
  }

  if (!fixedButtons.contains(existingImport)) fixedButtons.appendChild(existingImport);
}

// Modify the initialization to ensure DOM is loaded
function initializeUI() {
  if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', setupButtons);
  } else {
    setupButtons();
  }
}

function setupButtons() {
  updateAll();
  setInterval(updateAll, 500);
}

function importZipFile(file) {
  const jszip = new JSZip();
  jszip.loadAsync(file).then((zip) => {
    // mapping of expected zip paths to form names
    const mapping = {
      'swerve/controllerproperties.json': 'controllerproperties',
      'swerve/swervedrive.json': 'swervedrive',
      'swerve/modules/physicalproperties.json': 'physicalproperties',
      'swerve/modules/frontleft.json': 'frontleft',
      'swerve/modules/frontright.json': 'frontright',
      'swerve/modules/backleft.json': 'backleft',
      'swerve/modules/backright.json': 'backright',
      'swerve/modules/pidfproperties.json': 'pidfproperties'
    };

    // try each expected file; when present, load and populate
    Object.keys(mapping).forEach((path) => {
      const entry = zip.file(path);
      if (entry) {
        entry.async('string').then((content) => {
          try {
            const obj = JSON.parse(content);
            const name = mapping[path];
            // update JSON textarea display
            $(`#${name}-json`).text(JSON.stringify(obj, null, 2));
            // populate form inputs where possible
            populateFormFromObject(name, obj);
            updateAll();
          } catch (e) {
            console.error('Failed to parse JSON from', path, e);
          }
        });
      }
    });

  }).catch((err) => {
    console.error('Failed to read ZIP', err);
    alert('Failed to read ZIP file. See console for details.');
  });
}

// Populate a form identified by `name` from a parsed JSON object.
// Handles up to two levels of nesting to match the serialize naming convention
// used elsewhere in the app (struct_subStruct_field).
function populateFormFromObject(name, obj) {
  if (!obj || typeof obj !== 'object') return;
  const $form = $(`#${name}-form`);
  if ($form.length === 0) return;

  function setInputValue($input, value) {
    if (!$input.length) return;
    if ($input.attr('type') === 'checkbox') {
      $input.prop('checked', !!value);
    } else {
      // Always remove data-is-null first
      $input.removeAttr('data-is-null');
      
      // Special handling for CAN bus fields
      const isCanBusField = $input.attr('name').toLowerCase().includes('canbus');
      if (isCanBusField && value === null) {
        // For null CAN bus values, set empty value but mark as null
        $input.val('');
        $input.attr('data-is-null', 'true');
      } else {
        // For all other fields (including non-null CAN bus)
        const finalValue = (value === null || value === undefined || value === '') ? '' : value;
        $input.val(finalValue);
      }
    }
  }

  function processNestedObject(baseKey, nestedObj) {
    Object.entries(nestedObj).forEach(([key, value]) => {
      const fullKey = baseKey ? `${baseKey}_${key}` : key;
      if (value !== null && typeof value === 'object') {
        // Recursively process nested objects
        processNestedObject(fullKey, value);
      } else {
        const $input = $form.find(`[name="${fullKey}"]`);
        setInputValue($input, value);
      }
    });
  }

  processNestedObject('', obj);
}

$(function () {
  const tooltipTriggerList = document.querySelectorAll(
      '[data-bs-toggle="tooltip"]'); // Initialize tooltips: https://getbootstrap.com/docs/5.3/components/tooltips/#enable-tooltips
  const tooltipList = [...tooltipTriggerList].map(
      tooltipTriggerEl => new bootstrap.Tooltip(tooltipTriggerEl));

  $('a').click(function () {
    window.open(this.href, '_blank');
    return false;
  });

  // Single initialization point for all UI elements
  addZipImporter();
  setupButtons();
});