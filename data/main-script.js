window.addEventListener("load", getReadings);

function getReadings() {
    console.log("getReadings");
    var xhr = new XMLHttpRequest;
    xhr.onreadystatechange = function() {
        if (this.readyState == 4 && this.status == 200) {
            var myObj = JSON.parse(this.responseText);
            //document.getElementById("mastRotate").innerHTML = myObj.mastRotate;
        }
    };
    xhr.open("GET", "/readings", true);
    xhr.send()
}
if (!!window.EventSource) {
    var source = new EventSource("/events");
    source.addEventListener("open", function(e) {
        console.log("Events Connected")
    }, false);
    source.addEventListener("error", function(e) {
        if (e.target.readyState != EventSource.OPEN) {
            console.log("Events Disconnected")
        }
    }, false);
    source.addEventListener("message", function(e) {
        console.log("message", e.data)
    }, false);
    source.addEventListener("new_readings", function(e) {
        console.log("new_readings", e.data);
        const myObj = JSON.parse(e.data);
        // Define directions and their corresponding object properties
        const directions = [
            { id: 'N', sigProperty: 'N' },
            { id: 'S', sigProperty: 'S' },
            { id: 'E', sigProperty: 'E' },
            { id: 'W', sigProperty: 'W' }
        ];
        // Define signal colors and their image mappings
        const signalImages = {
            'Y': 'yellow-sig.png',
            'G': 'green-sig.png',
            'R': 'red-sig.png',
            'default': ''
        };
        // Process each direction
        directions.forEach(direction => {
            const element = document.getElementById(direction.id);
            if (element) {
                const signalValue = myObj[direction.sigProperty];
                element.innerHTML = signalValue;
                
                //console.log(`${direction.id}: ${signalValue}`);
                element.src = signalImages[signalValue] || signalImages.default;
                //console.log(element.src);
            } else {
                console.log(`no ${direction.id} image`);
            }
        });
    }, false);
}

function menuFun() {
    var x = document.getElementById("myLinks");
    if (x.style.display === "block") {
        x.style.display = "none"
    } else {
        x.style.display = "block"
    }
}