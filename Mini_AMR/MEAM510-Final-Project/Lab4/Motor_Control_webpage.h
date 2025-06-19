const char body[] PROGMEM = R"===(
<!DOCTYPE html>
<html>
<body>
    <h1> Control Motor </h1>
    <p> Control Speed:
        <input type="range" min="50" max="100" value="60" id="duty" onchange="updateDutyLabel(this.value)">
        <span id="outputlabelDuty">100</span> <br>
    </p>
    <p> Change Direction:
        <input type="range" min="0" max="1" value="0" step="1" id="directionToggle" onchange="updateDirectionLabel(this.value)">
        <span id="outputlabelToggle">Forward</span> <br>
    </p>
</body>

<script>
    function updateDutyLabel(value) {
        document.getElementById("outputlabelDuty").innerHTML = value;
        var xhttp = new XMLHttpRequest();
        xhttp.onreadystatechange = function() {
            if (this.readyState === 4 && this.status === 200) {
                document.getElementById("outputlabelDuty").innerHTML = this.responseText;
            }
        };
        var str = "duty?val=" + value;
        xhttp.open("GET", str, true);
        xhttp.send();
    }

    function updateDirectionLabel(value) {
    var directionValue = value === "1" ? 1 : 0;  // 1 for "Forward", 0 for "Backward"
    var directionText = (directionValue === 1) ? "Forward" : "Backward"; // Display text based on directionValue
    document.getElementById("outputlabelToggle").innerHTML = directionText;
    
    var xhttp = new XMLHttpRequest();
    xhttp.onreadystatechange = function() {
        if (this.readyState === 4 && this.status === 200) {
            // Handle the response if necessary
            document.getElementById("outputlabelToggle").innerHTML = this.responseText;
        }
    };
    
    var res = "changeDirection?val=" + directionValue;
    xhttp.open("GET", res, true);
    xhttp.send();
}
</script>
</html>
)===";

