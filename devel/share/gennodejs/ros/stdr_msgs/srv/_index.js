
"use strict";

let DeleteThermalSource = require('./DeleteThermalSource.js')
let LoadMap = require('./LoadMap.js')
let RegisterGui = require('./RegisterGui.js')
let AddRfidTag = require('./AddRfidTag.js')
let MoveRobot = require('./MoveRobot.js')
let DeleteCO2Source = require('./DeleteCO2Source.js')
let AddCO2Source = require('./AddCO2Source.js')
let AddSoundSource = require('./AddSoundSource.js')
let LoadExternalMap = require('./LoadExternalMap.js')
let DeleteRfidTag = require('./DeleteRfidTag.js')
let DeleteSoundSource = require('./DeleteSoundSource.js')
let AddThermalSource = require('./AddThermalSource.js')

module.exports = {
  DeleteThermalSource: DeleteThermalSource,
  LoadMap: LoadMap,
  RegisterGui: RegisterGui,
  AddRfidTag: AddRfidTag,
  MoveRobot: MoveRobot,
  DeleteCO2Source: DeleteCO2Source,
  AddCO2Source: AddCO2Source,
  AddSoundSource: AddSoundSource,
  LoadExternalMap: LoadExternalMap,
  DeleteRfidTag: DeleteRfidTag,
  DeleteSoundSource: DeleteSoundSource,
  AddThermalSource: AddThermalSource,
};
