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
      $("#freespeedrpm-input").val(5676);
    case "falcon":
      $("#freespeedrpm-input").val(6800);
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
      let struct = val.name.substring(0, val.name.indexOf('_'));
      if (data[struct] === undefined) {
        data[struct] = {};
      }
      data[struct][val.name.substring(val.name.indexOf('_') + 1)] =
          isNumeric(val.value) ?
              parseFloat(val.value) :
              (val.value === "" ? null : val.value);
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