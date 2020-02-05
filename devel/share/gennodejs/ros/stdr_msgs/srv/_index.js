
"use strict";

let LoadMap = require('./LoadMap.js')
let MoveRobot = require('./MoveRobot.js')
let AddCO2Source = require('./AddCO2Source.js')
let DeleteRfidTag = require('./DeleteRfidTag.js')
let AddThermalSource = require('./AddThermalSource.js')
let DeleteCO2Source = require('./DeleteCO2Source.js')
let AddRfidTag = require('./AddRfidTag.js')
let AddSoundSource = require('./AddSoundSource.js')
let DeleteThermalSource = require('./DeleteThermalSource.js')
let DeleteSoundSource = require('./DeleteSoundSource.js')
let LoadExternalMap = require('./LoadExternalMap.js')
let RegisterGui = require('./RegisterGui.js')

module.exports = {
  LoadMap: LoadMap,
  MoveRobot: MoveRobot,
  AddCO2Source: AddCO2Source,
  DeleteRfidTag: DeleteRfidTag,
  AddThermalSource: AddThermalSource,
  DeleteCO2Source: DeleteCO2Source,
  AddRfidTag: AddRfidTag,
  AddSoundSource: AddSoundSource,
  DeleteThermalSource: DeleteThermalSource,
  DeleteSoundSource: DeleteSoundSource,
  LoadExternalMap: LoadExternalMap,
  RegisterGui: RegisterGui,
};
