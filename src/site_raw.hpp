#pragma once


constexpr auto site = R"htmlcode(
<html>
<head>
  <script src="https://cdn.jsdelivr.net/npm/@jaames/iro/dist/iro.min.js"></script>
  <style>
      #colors {
          display: flex;
          align-items: center;
          align-content: center;
          box-sizing: border-box;
      }
      #colors > * {
          padding: 4px;
      }

      button {
          padding: 8px 16px;
          width: 100%;
      }

      #root {
          display: flex;
          flex-direction: column;
          align-content: center;
          align-items: center;
          max-width: 500px;
          background: white;
          border-radius: 4px;
          margin: auto;
          padding: 10px;
      }

      body {
          background: #eee;
          overflow: auto;
      }
  </style>
</head>
<body>
    <script>
      const send = (val) => {
        var req = new XMLHttpRequest();
        req.open("GET", `/?${val}`);
        req.send();
      }
    </script>

    <div id="root">
        <div id="colors">
            <div id="color-picker-container1"></div>
            <div id="color-picker-container2"></div>
        </div>
        <input type="number" id="brightness" value="20" min="1" max="255"/>
        <button id="apply">Apply</button>

        <div><input id="cal_base" type="checkbox" title="Calibrate Baselines" onchange="send(`cal_base=${checked?1:0}`)"/><label for="cal_base">Calibrate Baselines</label></div>
        <div><input id="cal_tops" type="checkbox" title="Calibrate Tops" onchange="send(`cal_tops=${checked?1:0}`)"/><label for="cal_tops">Calibrate Tops</label></div>
    </div>

  <script>
    var colorPicker1 = new iro.ColorPicker("#color-picker-container1", {
      // Set the size of the color picker
      width: 160,
      // Set the initial color to pure red
      color: "#f00"
    });
    var colorPicker2 = new iro.ColorPicker("#color-picker-container2", {
      // Set the size of the color picker
      width: 160,
      // Set the initial color to pure red
      color: "#0f0"
    });

    function HEXToVBColor(rrggbb) {
        return parseInt(rrggbb.substr(1, 6), 16);
    }

    document.getElementById('apply').onclick = function(){
      send("color1=" + HEXToVBColor(colorPicker1.color.hex8String) + "&color2=" + HEXToVBColor(colorPicker2.color.hex8String) + "&bright=" + document.getElementById("brightness").value);

      console.log("Applied");
    }
  </script>
</body>
</html>
)htmlcode";