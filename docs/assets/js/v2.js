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
        data[struct][val.name.substring(val.name.lastIndexOf('_') + 1)] =
            isNumeric(val.value) ?
                parseFloat(val.value) :
                (val.value === "" ? null : val.value);
      } else {
        data[struct][subStruct][val.name.substring(
            val.name.lastIndexOf('_') + 1)] =
            isNumeric(val.value) ?
                parseFloat(val.value) :
                (val.value === "" ? null : val.value);
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

$(function () {
  const tooltipTriggerList = document.querySelectorAll(
      '[data-bs-toggle="tooltip"]'); // Initialize tooltips: https://getbootstrap.com/docs/5.3/components/tooltips/#enable-tooltips
  const tooltipList = [...tooltipTriggerList].map(
      tooltipTriggerEl => new bootstrap.Tooltip(tooltipTriggerEl));

  $('a').click(function () {
    window.open(this.href, '_blank');
    return false;
  });

  updateAll();
  setInterval(updateAll, 500);
});