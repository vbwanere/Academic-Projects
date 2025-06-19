// LED Control webpage //

const char body[] PROGMEM = R"===(
<!DOCTYPE html>
<html>
<body>
    <h1> Control LED <br>
    <input type="range" min="3" max="30" value="15" id="freq">
    <span id="outputlabelFreq"> </span> <br>
    <input type="range" min="1" max="4095" value="2000" id="duty">
    <span id="outputlabelDuty"> </span> <br>
</body>

<script>
    document.getElementById('freq').onchange = function() {
        var xhttp = new XMLHttpRequest();
        xhttp.onreadystatechange = function() {
            if (this.readyState == 4 && this.status == 200) {
                document.getElementById("outputlabelFreq").innerHTML = this.responseText;
            }
        };
        var str = "freq?val=";
        var res = str.concat(this.value);
        xhttp.open("GET", res, true);
        xhttp.send();
    }

    document.getElementById('duty').onchange = function() {
        var xhttp = new XMLHttpRequest();
        xhttp.onreadystatechange = function() {
            if (this.readyState == 4 && this.status == 200) {
                document.getElementById("outputlabelDuty").innerHTML = this.responseText;
            }
        };
        var str = "duty?val=";
        var res = str.concat(this.value);
        xhttp.open("GET", res, true);
        xhttp.send();
    }
</script>
)===";

