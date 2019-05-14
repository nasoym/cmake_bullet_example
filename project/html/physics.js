var scene, camera, renderer, raycaster, mouse;

function animate() {
  requestAnimationFrame(animate);
  renderer.render(scene, camera);
  controls.update();
  stats.update();
}

function render() {
  renderer.render( scene, camera );
}
// var rightmousemove;
// document.addEventListener("mousedown", function(event){
//       rightmousemove = false;
//       if(event.button == 2){
//         rightmousemove = true;
//         return false;
//       //   // Right click
//       } 
//     });
//     document.addEventListener("mousemove", function(event){
//       // if(rightmousemove === true){
//       	// Use stopImmediatePropagation to stop the other handeller from trigerring 
//         // event.stopImmediatePropagation();
//       // }
//     });


var clicked_body_id = null;
function onDocumentMouseClick( event ) {
    event.preventDefault();
    // event.stopImmediatePropagation();
    mouse.x = ( event.clientX / window.innerWidth ) * 2 - 1;
    mouse.y = - ( event.clientY / window.innerHeight ) * 2 + 1;
    raycaster.setFromCamera( mouse, camera );
    var intersects = raycaster.intersectObjects( scene.children );
    clicked_body_id = null;
    if ( intersects.length > 0 ) {
      var object = intersects[ 0 ].object;
      if ( ('bullet_type' in object) && (object.bullet_type === "physic_body" )){
        console.log("clicked object: ", object.bullet_id);
        clicked_body_id = object.bullet_id;
      }
    }
  }
function onDocumentMouseMove( event ) {
  if (clicked_body_id == null) {
    // console.log("mouse move none");
  } else {
    // console.log("mouse move id: ", clicked_body_id);
  }
  // console.log("event.button: " , event.button);
}
function onDocumentMouseDown( event ) {
  console.log("mouse down");
  // console.log("event.button: " , event.button);
      // if(event.button == 2){
}
function onDocumentMouseUp( event ) {
  console.log("mouse up");
  clicked_body_id = null;
  // event.preventDefault();
  // event.stopImmediatePropagation();
      // if(event.button == 2){
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
  camera.position.set(0,-20,15);
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
  light.position.set( 20, -50, 80 ); 			//default; light shining from top
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

  var ambient_light = new THREE.AmbientLight(0xffffff, 0.8);
  scene.add(ambient_light);


  //Create a helper for the shadow camera (optional)

  // var helper = new THREE.CameraHelper( light.shadow.camera );
  // scene.add( helper );

  // controls = new THREE.OrbitControls(camera, renderer.domElement);

  document.addEventListener( 'mousemove', onDocumentMouseMove, false );
  window.addEventListener( 'mousedown', onDocumentMouseDown, false );
  document.addEventListener( 'mouseup', onDocumentMouseUp, false );

  controls = new THREE.TrackballControls(camera , renderer.domElement);
  controls.rotateSpeed = 1.0;
  controls.zoomSpeed = 3.0;
  controls.panSpeed = 1.5;
  controls.noZoom = false;
  controls.noPan = false;
  controls.staticMoving = true;
  controls.dynamicDampingFactor = 0.3;
  controls.keys = [ 65, 83, 68 ];
  controls.addEventListener( 'change', render );

  stats = new Stats();
  document.body.appendChild( stats.dom );

  raycaster = new THREE.Raycaster();
  mouse = new THREE.Vector2();
  // window.addEventListener( 'click', onDocumentMouseClick, false );
  // document.addEventListener( 'click', onDocumentMouseClick, false );
  // window.addEventListener( 'dblclick', onDocumentMouseClick, false );


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


  function addTexture(text,bg_color,fontcolor,fontsize,centerText,bitmap_width,bitmap_height) {
    //create image
    var bitmap = createRetinaCanvas(bitmap_width, bitmap_height);
    var ctx = bitmap.getContext('2d', {antialias: false});
    ctx.font = 'Bold '+fontsize+'px Arial';

    // ctx.globalAlpha= 0;

    ctx.beginPath();
    ctx.rect(0, 0, bitmap_width, bitmap_height);
    // ctx.fillStyle = 'green';
    ctx.fillStyle = bg_color;
    ctx.fill();


    // ctx.textAlign = textAlign;
    // ctx.textBaseline = "middle";
    // ctx.rotate( Math.PI / 4 );
    // ctx.rotate( Math.PI / 2 );


    ctx.globalAlpha= 1;
    ctx.fillStyle = fontcolor;
    // ctx.fillText(text, 0, 20);
    // ctx.fillText(text, 0, 50);

    if (centerText) {

      ctx.textAlign = 'center';
      ctx.textBaseline = 'middle';
      printAt(ctx, text, bitmap_width/2, bitmap_height/2, fontsize, bitmap_width );
    } else {
      ctx.textBaseline = 'top';
      printAt(ctx, text, 0, 0, fontsize, bitmap_width );

    }


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
  var bg_color = "rgb(50%,70%,50%)";
  var opacity = 1.0;
  var transparent = false;
  var wireframe = false;
  if ( (data.hasOwnProperty("json")) && (data["json"].hasOwnProperty("color")) ){
    bg_color = data["json"]["color"];
  }
  if ( (data.hasOwnProperty("json")) && (data["json"].hasOwnProperty("opacity")) ){
    opacity = data["json"]["opacity"];
  }
  if ( (data.hasOwnProperty("json")) && (data["json"].hasOwnProperty("wireframe")) ){
    wireframe = data["json"]["wireframe"];
  }
  if ( opacity < 1.0 ) {
    transparent = true;
  }
  // var material = new THREE.MeshPhongMaterial({color: bg_color});
  // var material = new THREE.MeshLambertMaterial({color: bg_color});
  var material = new THREE.MeshStandardMaterial({color: bg_color, transparent: transparent, opacity: opacity});
  // var material = new THREE.MeshBasicMaterial({color: bg_color});

  // var material_color = new THREE.MeshBasicMaterial({color: 0x55B663, wireframe: false});
  var material_wireframe = new THREE.MeshBasicMaterial({color: bg_color, wireframe: true, wireframeLinewidth:3, transparent: transparent, opacity: opacity});

  var material_array = [];
  material_array = [material,material,material,material,material,material];
  if ( (data.hasOwnProperty("json")) && (data["json"].hasOwnProperty("text")) ){

    var fontcolor = 'white';
    var fontsize = 20;
    var centerText = false;
    var bitmap_width = 1.0;
    var bitmap_height = 1.0;
    var bitmap_pixel_per_unit = 64;
    if ( (data["json"].hasOwnProperty("fontcolor")) ){
      fontcolor = data["json"]["fontcolor"];
    }
    if ( (data["json"].hasOwnProperty("fontsize")) ){
      fontsize = data["json"]["fontsize"];
    }
    if ( (data["json"].hasOwnProperty("centerText")) ){
      centerText = data["json"]["centerText"];
    }
    if (data.hasOwnProperty("size")) {
      bitmap_width *= data["size"][0];
      bitmap_height *= data["size"][2];
    }
    if ( (data["json"].hasOwnProperty("pixel_per_unit")) ){
      bitmap_pixel_per_unit = data["json"]["pixel_per_unit"];
    }

    var material_text = new THREE.MeshStandardMaterial({ map: addTexture(data["json"]["text"],bg_color,fontcolor,fontsize,centerText,bitmap_width* bitmap_pixel_per_unit,bitmap_height* bitmap_pixel_per_unit) });
    if ( (data["json"].hasOwnProperty("textSides")) ){
      var textSides = data["json"]["textSides"];
      for (i in textSides) {
        material_array[textSides[i]] = material_text;
      }
    } else {
      // material_array = [material,material,material_text,material_text,material,material];
      material_array[3] = material_text;
    }



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
      if (wireframe) {
        body = new THREE.Mesh(geometry, material_wireframe);
      } else {
        body = new THREE.Mesh(geometry, material_array);
      }
          // [material_array,material_wireframe]
      var castShadow = true;
      var receiveShadow = true;
      if ( (data.hasOwnProperty("json")) && (data["json"].hasOwnProperty("castShadow")) ){
        castShadow = data["json"]["castShadow"];
      }
      if ( (data.hasOwnProperty("json")) && (data["json"].hasOwnProperty("receiveShadow")) ){
        receiveShadow = data["json"]["receiveShadow"];
      }
      body.castShadow = castShadow;
      body.receiveShadow = receiveShadow;

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

