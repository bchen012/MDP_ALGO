document.addEventListener('DOMContentLoaded', function initialize(e) {
    var map = [];
    for (i=0; i<20; i++){
        map[i]=[];
        for (j=0;j<15;j++){
            map[i][j]=0;
        }
    }
    var canvas = document.getElementById('myCanvas');
    var context = canvas.getContext('2d');
    context.save();
    context.strokeStyle = "#252a33";
    context.lineWidth = 3;
    

    function getStyle(cell) {
	    switch(cell) {
	        case 0: return "#1a1e24"; // unexplored
	        case 1: return "#F9F0DD"; // explored
	        case 2: return "#003BCB"; // obstacle
	        case 3: return "#30807d"; // start
	        case 4: return "#08ae69"; // goal
	        case 5: return "#354458"; // robot
	        case 6: return "#7acdc8"; // path
	        case 7: return "#673ab7"; // way-point
	        default: return "#1a1e24";
	    }
	}
	
	function updateGUI(map, center=null, head=null,roboPath=null) {
		context.save();
        context.strokeStyle = "#252a33";
        context.lineWidth = 3;

        // Filling explored and unexplored cells
        for (var i = 0; i < 20; i++){
			for (var j = 0; j < 15; j++){
				context.beginPath();
				context.fillStyle = getStyle(map[i][j]);
				context.rect(30 * j, 30 * i, 30, 30);
				context.fill();
				context.stroke();
				context.closePath();
			}
		}

		// add path
		if (roboPath){
			for(var i=0; i<roboPath.length; i++){
				pt = roboPath[i];
				if (map[pt[0]][pt[1]] != 7){
					context.beginPath();
					context.fillStyle = getStyle(6);
					context.rect(30 * pt[1], 30 * pt[0], 30, 30);
					context.fill();
					context.stroke();
					context.closePath();
				}
			}
		}

		if (center && head){
			context.beginPath();
			context.fillStyle = getStyle(5);
			context.moveTo(30*center[1] + 55, 30*center[0] + 15);
			context.arc(30*center[1] + 15, 30*center[0] + 15, 40, 0, 2 * Math.PI, true);
			context.fill();
			context.stroke();
			context.closePath();
			context.beginPath();
			context.fillStyle = getStyle(6);
			context.moveTo(30*head[1] + 20, 30*head[0] + 20);
			context.arc(30*head[1] + 15, 30*head[0] + 20, 5, 0, 2 * Math.PI, true);
			context.fill();
			context.closePath();
		}
        context.restore();
	}

	document.getElementById('start').addEventListener('click', function(e){
		var r = new XMLHttpRequest();
		r.open("GET", "/start");
		r.send();
	});

	document.getElementById('FSP').addEventListener('click', function(e){
		var r = new XMLHttpRequest();
		r.open("GET", "/fsp");
		r.send();
	});
    
    function wsConnect() {
		this.ws = new WebSocket("ws://"+window.location.host+"/websocket?Id=" + Math.floor(Math.random() * 100));
	    this.ws.onopen = function() {
	        ws.send("Initializing connection");
	    };
	    this.ws.onmessage = function(evt){
	    	var data = JSON.parse(evt.data);
	    	if (data.hasOwnProperty('log')){
	    		var msg = data.log;
	    		console.log(msg);
	    		//log(msg)
	    	}
	    	else{
	    		var map = JSON.parse(data.map);
		    	var center = JSON.parse(data.center);
		    	var head = JSON.parse(data.head);
		    	// var area = data.area;
		    	// var time = data.time;
		    	// var msg = data.msg;
		    	// roboPath.push(center);
		    	updateGUI(map,center,head);
	    	}
	    };
	    this.ws.onerror = function(evt){
	    	console.log('WebSocket Error: ' + error);
	    }
	    this.ws.onclose = function(evt){
	    	setTimeout(wsConnect, 1000);
	    	console.log(evt);
	    }
    }
    updateGUI(map)
    wsConnect()
});

