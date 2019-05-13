var scene, camera, renderer;

function animate() {
  requestAnimationFrame(animate);
  renderer.render(scene, camera);
  controls.update();
  stats.update();
}

function render() {
  renderer.render( scene, camera );
}

function init() {
  scene = new THREE.Scene();
  var WIDTH = window.innerWidth,
      HEIGHT = window.innerHeight;

  renderer = new THREE.WebGLRenderer({antialias:true});
  renderer.setSize(WIDTH, HEIGHT);
  renderer.shadowMap.enabled = true;
  renderer.shadowMap.type = THREE.PCFSoftShadowMap; // default THREE.PCFShadowMap
  // renderer.shadowMap.type = THREE.BasicShadowMap;

  document.body.appendChild(renderer.domElement);

  camera = new THREE.PerspectiveCamera(45, WIDTH / HEIGHT, 0.1, 20000);
  // camera.rotateOnAxis(new THREE.Vector3(1, 0, 0), degInRad(90));
  // camera.rotation.order = 'YXZ';
  camera.position.set(30,30,30);
  camera.up = new THREE.Vector3(0,0,1);
  camera.lookAt(new THREE.Vector3(0,0,0));
  scene.add(camera);

  // Create an event listener that resizes the renderer with the browser window.
  window.addEventListener('resize', function() {
    var WIDTH = window.innerWidth,
        HEIGHT = window.innerHeight;
    renderer.setSize(WIDTH, HEIGHT);
    camera.aspect = WIDTH / HEIGHT;
    camera.updateProjectionMatrix();
  });

  // renderer.setClearColor(new THREE.Color(0x333F47, 1));
  // renderer.setClearColor(new THREE.Color("rgb(50%,50%,50%)", 1));
  // renderer.setClearColor(0xffffff);
  renderer.setClearColor(0x888888,1);

  var light = new THREE.DirectionalLight( 0xffffff, 1, 100 );
  light.position.set( 50, 0, 80 ); 			//default; light shining from top
  light.target.position.set(0, 0, 0);
  light.castShadow = true;            // default false
  //Set up shadow properties for the light
  light.shadow.mapSize.width = 512 * 4;  // default
  light.shadow.mapSize.height = 512 * 4; // default
  light.shadow.camera.near = 0.5;    // default
  light.shadow.camera.far = 500;     // default

  light.shadow.camera.right = 35;
  light.shadow.camera.left = - 35;
  light.shadow.camera.top	= 35;
  light.shadow.camera.bottom = - 35;
  
  scene.add(light);

  var ambient_light = new THREE.AmbientLight(0xffffff, 0.5);
  scene.add(ambient_light);


  //Create a helper for the shadow camera (optional)

  // var helper = new THREE.CameraHelper( light.shadow.camera );
  // scene.add( helper );

  // controls = new THREE.OrbitControls(camera, renderer.domElement);

  controls = new THREE.TrackballControls(camera , renderer.domElement);
  controls.rotateSpeed = 1.0;
  controls.zoomSpeed = 1.2;
  controls.panSpeed = 0.8;
  controls.noZoom = false;
  controls.noPan = false;
  controls.staticMoving = true;
  controls.dynamicDampingFactor = 0.3;
  controls.keys = [ 65, 83, 68 ];
  controls.addEventListener( 'change', render );

  stats = new Stats();
  document.body.appendChild( stats.dom );

}

var PIXEL_RATIO = (function () {
    var ctx = document.createElement('canvas').getContext('2d'),
        dpr = window.devicePixelRatio || 1,
        bsr = ctx.webkitBackingStorePixelRatio ||
              ctx.mozBackingStorePixelRatio ||
              ctx.msBackingStorePixelRatio ||
              ctx.oBackingStorePixelRatio ||
              ctx.backingStorePixelRatio || 1;
    return dpr / bsr;
  })();


  createRetinaCanvas = function(w, h, ratio) {
    if (!ratio) { ratio = PIXEL_RATIO; }
    var can = document.createElement('canvas');
    can.width = w * ratio;
    can.height = h * ratio;
    can.style.width = w + 'px';
    can.style.height = h + 'px';
    can.getContext('2d').setTransform(ratio, 0, 0, ratio, 0, 0);
    return can;
  }

function printAt( context , text, x, y, lineHeight, fitWidth)
{
    fitWidth = fitWidth || 0;
    
    if (fitWidth <= 0)
    {
         context.fillText( text, x, y );
        return;
    }
    
    for (var idx = 1; idx <= text.length; idx++)
    {
        var str = text.substr(0, idx);
        // console.log(str, context.measureText(str).width, fitWidth);
        if (context.measureText(str).width > fitWidth)
        {
            context.fillText( text.substr(0, idx-1), x, y );
            printAt(context, text.substr(idx-1), x, y + lineHeight, lineHeight,  fitWidth);
            return;
        }
    }
    context.fillText( text, x, y );
}


  function addTexture(text,color) {
    //create image
    var bitmap = createRetinaCanvas(100, 100);
    var ctx = bitmap.getContext('2d', {antialias: false});
    ctx.font = 'Bold 40px Arial';

    // ctx.globalAlpha= 0;

    ctx.beginPath();
    ctx.rect(0, 0, 100, 100);
    // ctx.fillStyle = 'green';
    ctx.fillStyle = color;
    ctx.fill();

    // ctx.textAlign = "center";
    // ctx.textBaseline = "middle";


    ctx.globalAlpha= 1;
    ctx.fillStyle = 'white';
    // ctx.fillText(text, 0, 20);
    // ctx.fillText(text, 0, 50);
    printAt(ctx, text, 0, 40, 40, 100 );


    // var metrics = ctx.measureText(text);
    // console.log("text measure: " , metrics );
    
    // ctx.strokeStyle = 'black';
    // ctx.strokeText(text, 0, 20);
    // printAt(ctx, text, 0, 20, 20, 100 );

    // canvas contents will be used for a texture
    var texture = new THREE.Texture(bitmap) 
    texture.needsUpdate = true;
    return texture;
  }


function create_body(data) {
  var id = data["id"];
  // var material = new THREE.MeshLambertMaterial({color: 0x55B663});
  // var color = 0x55B663;
  var color = "rgb(50%,70%,50%)";
  if ( (data.hasOwnProperty("json")) && (data["json"].hasOwnProperty("color")) ){
    color = data["json"]["color"];
  }
  // var material = new THREE.MeshPhongMaterial({color: color});
  // var material = new THREE.MeshLambertMaterial({color: color});
  var material = new THREE.MeshStandardMaterial({color: color});
  // var material = new THREE.MeshBasicMaterial({color: color});

  // var material_color = new THREE.MeshBasicMaterial({color: 0x55B663, wireframe: false});
  var material_wireframe = new THREE.MeshBasicMaterial({color: 0x050603, wireframe: true, wireframeLinewidth:3});

  var material_array = [];
  if ( (data.hasOwnProperty("json")) && (data["json"].hasOwnProperty("text")) ){
    var material_text = new THREE.MeshStandardMaterial({ map: addTexture(data["json"]["text"],color) });
    material_array = [material,material,material_text,material_text,material,material];

// var materials = [
//     leftSide,        // Left side
//     rightSide,       // Right side
//     topSide,         // Top side
//     bottomSide,      // Bottom side
//     frontSide,       // Front side
//     backSide         // Back side
// ];

  } else {
    material_array = [material,material,material,material,material,material];
  }

  // console.log("create body with id:", id , " data: ", data);
  console.log("create body with id:", id);

  if (data.hasOwnProperty("type")) {
    var type = data["type"];
    if (type === "box" ) {
      // console.log("create box");
      var geometry = new THREE.CubeGeometry( 1, 1, 1 );
      // body = THREE.SceneUtils.createMultiMaterialObject( 
      //     geometry,
      //     [material]
      //   );
      body = new THREE.Mesh(geometry, material_array);
          // [material_array,material_wireframe]
      if ( 
          (data.hasOwnProperty("json")) && 
          (data["json"].hasOwnProperty("cast")) && 
          (data["json"]["cast"] == 1)
        ){
        body.castShadow = false;
      } else {
        body.castShadow = true;
      }
      body.receiveShadow = true;

    } else if (type === "plane" ) {
      console.log("create plane");
      // 2nd and 3rd argument are the vertical / horizontal segments
      body = THREE.SceneUtils.createMultiMaterialObject( 
          new THREE.PlaneGeometry( 1, 1 ), 
          [material,material_wireframe]
        );
          // [material_color,material_wireframe]
    // } else if (type === "sphere" ) {
    }
  }
  body.bullet_id = id;
  body.bullet_type = "physic_body";
  scene.add( body );
  return body;
}

function update_body(data) {
  var body = scene.getObjectByProperty("bullet_id", data["id"]);
  if (typeof body === "undefined") {
    body = create_body(data);
  }
  if (data.hasOwnProperty("pos")) {
    body.position.set(data["pos"][0],data["pos"][1],data["pos"][2]);
  }
  if (data.hasOwnProperty("rot")) {
    // console.log(data["rot"][0],data["rot"][1],data["rot"][2],data["rot"][3]);
    body.quaternion.set(data["rot"][0],data["rot"][1],data["rot"][2],data["rot"][3]);
  }
  if (data.hasOwnProperty("size")) {
    body.scale.set(data["size"][0],data["size"][1],data["size"][2]);
  }
}

function update_bodies(data) {
  var all_ids = [];
  scene.traverse (function (object) {
    if ( ('bullet_type' in object) && (object.bullet_type === "physic_body" )){
      all_ids.push(object.bullet_id);
    }
  });
  var id;
  for (i in data) {
    update_body(data[i]);
    id = data[i]["id"];
    all_ids = all_ids.filter(function(val,i,a) {return val !== id;})
  }

  // console.log("unused ids: ",all_ids);
  for (i in all_ids) {
    console.log("remove : ",all_ids[i]);
    scene.remove(scene.getObjectByProperty("bullet_id", all_ids[i]));
  }
}

function setup_update_listener(address,exchange_name) {
  // Stomp.js boilerplate
  var client = Stomp.client('ws://' + address + '/ws');
  client.debug = function(a,b) { };

  var on_connect = function(x) {
    id = client.subscribe("/exchange/"+exchange_name, function(d) {
      update_bodies(JSON.parse(d.body));
    });
  };
  var on_error =  function() {
    console.log('error: ', exchange_name);
  };
  client.connect('guest', 'guest', on_connect, on_error, '/');
}

function create_debug_body(data) {
  // var material = new THREE.MeshLambertMaterial({color: 0x55B663});
  var material_wireframe = new THREE.MeshBasicMaterial({color: 0xefefef, wireframe: true, wireframeLinewidth:3});

  var body = THREE.SceneUtils.createMultiMaterialObject( 
      new THREE.CubeGeometry( 1.5, 1.5, 1.5 ), 
      [material_wireframe]
    );
  body.bullet_type="debug_body";


  if (data.hasOwnProperty("pos")) {
    console.log("data has pos: " , data["pos"]);
    body.position.set(data["pos"][0],data["pos"][1],data["pos"][2]);
  }
  if (data.hasOwnProperty("rot")) {
    body.quaternion.set(data["rot"][0],data["rot"][1],data["rot"][2],data["rot"][3]);
  }
  if (data.hasOwnProperty("size")) {
    body.scale.set(data["size"][0],data["size"][1],data["size"][2]);
  }

  return body;
}

function debug_bodies(data) {
  var id;
  var body;
  var debug_body;
  for (i in data) {
    debug_body = create_debug_body(data[i]);
    if (data[i].hasOwnProperty("id")) {
      body = scene.getObjectByProperty("bullet_id", data[i]["id"]);
      if (typeof body !== "undefined") {
        body.add(debug_body);
      }
    } else {
      scene.add( debug_body );
    }
  }
}

function setup_debug_listener(address,exchange_name) {
  // Stomp.js boilerplate
  var client = Stomp.client('ws://' + address + '/ws');
  client.debug = function(a,b) { };

  console.log("setup debug listener");
  var on_connect = function(x) {
    id = client.subscribe("/exchange/"+exchange_name, function(d) {
      debug_bodies(JSON.parse(d.body));
    });
  };
  var on_error =  function() {
    console.log('error: ', exchange_name);
  };
  client.connect('guest', 'guest', on_connect, on_error, '/');
}

document.onreadystatechange = function () {
  if (document.readyState === "complete") {

    var urlParams = new URLSearchParams(window.location.search);
    var host = window.location.hostname;
    var port = 15674;
    //var port = 8080;
    var exchange_name = "updates";
    if (urlParams.has('host')) {
      host = urlParams.get('host');
    }
    if (host === "" ) {
      host = "localhost";
    }
    if (urlParams.has('port')) {
      port = urlParams.get('port');
    }
    if (urlParams.has('exchange')) {
      exchange_name = urlParams.get('exchange');
    }

    init();
    animate();
    setup_update_listener(host + ":" + port, exchange_name);
    setup_debug_listener(host + ":" + port, "debug_bodies");
  }
};

