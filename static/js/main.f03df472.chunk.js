(this["webpackJsonppublic-console2"]=this["webpackJsonppublic-console2"]||[]).push([[0],{130:function(e,a){},140:function(e,a,t){},141:function(e,a,t){"use strict";t.r(a);var n=t(0),r=t.n(n),c=t(16),o=t.n(c),l=(t(81),t(11)),m=(t(82),t(69)),i=t(176),s=t(193),u=t(177),E=t(178),d=t(180),p=t(179),f=t(70),h=t(143),g=t(181),b=t(190),v=t(189),w=t(191),N=t(194),k=t(182),x=t(192),y=t(183),S=t(185),O=t(186),j=t(188),R=t(184),C=t(187),B=t(68),L=t.n(B),A=Object(m.a)({palette:{primary:{main:"#ffffff"},secondary:{main:"#ED4A70"},contrastThreshold:3,tonalOffset:.2},typography:{button:{textTransform:"none"}}}),G=Object(i.a)((function(e){return Object(s.a)({root:{flexGrow:1},menuButton:{marginRight:e.spacing(2)},title:{flexGrow:1},paper:{padding:e.spacing(2),textAlign:"center",color:e.palette.text.secondary,background:"#fdfdfd"},radioButtons:{maxHeight:300,minHeight:300,textAlign:"center",overflow:"auto"},info:{padding:e.spacing(2),textAlign:"center",color:e.palette.text.secondary,background:"#f6f6f6"},header:{color:e.palette.text.primary,background:"#fcfcfc"},footer:{textAlign:"left"},footerLink:{color:"#38B48B",marginRight:"1rem"},radioGroup:{textAlign:"left"}})})),U=new L.a({baseURL:"https://rocky-peak-54058.herokuapp.com"}),D=function(){var e=r.a.useState(void 0),a=Object(l.a)(e,2),t=a[0],c=a[1],o=r.a.useState(null),m=Object(l.a)(o,2),i=m[0],s=m[1],B=r.a.useState([]),L=Object(l.a)(B,2),D=L[0],I=L[1],T=r.a.useState(""),H=Object(l.a)(T,2),J=H[0],P=H[1],W=r.a.useState([]),z=Object(l.a)(W,2),Y=z[0],$=z[1],q=r.a.useState(""),F=Object(l.a)(q,2),K=F[0],M=F[1],Q=r.a.useState({}),V=Object(l.a)(Q,2),X=(V[0],V[1]),Z=r.a.useState("primary"),_=Object(l.a)(Z,2),ee=_[0],ae=_[1],te=r.a.useState("Connect"),ne=Object(l.a)(te,2),re=ne[0],ce=ne[1],oe=r.a.useState(null),le=Object(l.a)(oe,2),me=le[0],ie=le[1];Object(n.useEffect)((function(){void 0===t&&U.currentConnectionList().then((function(e){console.log(e.data),c(e.data.map((function(e){return e.uuid})))}))}));var se=G();return r.a.createElement("div",{className:"".concat(se.root," App")},r.a.createElement(u.a,{theme:A},r.a.createElement(E.a,{position:"static",className:se.header},r.a.createElement(p.a,null,r.a.createElement(d.a,null,r.a.createElement(f.a,{variant:"h5"},"Rowma Network Console")))),r.a.createElement(d.a,null,r.a.createElement(g.a,{container:!0,spacing:3,className:"py-8"},r.a.createElement(g.a,{item:!0,xs:12,sm:12,md:4},r.a.createElement(h.a,{className:se.paper},r.a.createElement("div",null,r.a.createElement(x.a,{component:"fieldset",className:se.radioButtons},r.a.createElement("div",{className:"my-4"},r.a.createElement(f.a,{variant:"h5"},"Select Your Robot","'","s UUID")),(!t||t&&0===t.length)&&r.a.createElement("p",null,"Robot not found..."),r.a.createElement(N.a,{"aria-label":"robots",name:"robots",value:i,onChange:function(e){s(e.target.value)},className:se.radioGroup},t&&t.map((function(e){return r.a.createElement(k.a,{value:e,control:r.a.createElement(w.a,null),label:e})}))))),r.a.createElement("div",null,r.a.createElement(y.a,{variant:"contained",color:ee,onClick:function(){U.connect(i).then((function(e){ie(e)})).catch((function(e){console.log(e)})),U.getRobotStatus(i).then((function(e){console.log(e.data),X(e.data),I(e.data.rosrunCommands),$(e.data.launchCommands),ae("secondary"),ce("Disconnect")}))}},re)))),r.a.createElement(g.a,{item:!0,xs:12,sm:12,md:4},r.a.createElement(h.a,{className:se.paper},r.a.createElement("div",null,r.a.createElement(x.a,{component:"fieldset",className:se.radioButtons},r.a.createElement("div",{className:"my-4"},r.a.createElement(f.a,{variant:"h5"},"Select a rosrun command")),r.a.createElement(N.a,{"aria-label":"rosrun",name:"rosrun",value:J,onChange:function(e){P(e.target.value)},className:se.radioGroup},D&&D.map((function(e){return r.a.createElement(k.a,{value:e,control:r.a.createElement(w.a,null),label:e})}))))),r.a.createElement("div",null,r.a.createElement(y.a,{variant:"contained",color:"primary",onClick:function(){U.runRosrun(me,i,J,"")}},"Execute")))),r.a.createElement(g.a,{item:!0,xs:12,sm:12,md:4},r.a.createElement(h.a,{className:se.paper},r.a.createElement("div",null,r.a.createElement(x.a,{component:"fieldset",className:se.radioButtons},r.a.createElement("div",{className:"my-4"},r.a.createElement(f.a,{variant:"h5"},"Select a roslaunch command")),r.a.createElement(N.a,{"aria-label":"roslaunch",name:"roslaunch",value:K,onChange:function(e){M(e.target.value)},className:se.radioGroup},Y&&Y.map((function(e){return r.a.createElement(k.a,{value:e,control:r.a.createElement(w.a,null),label:e})}))))),r.a.createElement("div",null,r.a.createElement(y.a,{variant:"contained",color:"primary",onClick:function(){U.runLaunch(me,i,K)}},"Execute")))),r.a.createElement(g.a,{item:!0,xs:12},r.a.createElement(h.a,{className:se.paper},r.a.createElement("div",null,r.a.createElement("p",null,"Send (Topic Selectbox) from (Robot Selectbox) to (Robot Selectbox)")))),r.a.createElement(g.a,{item:!0,xs:12},r.a.createElement(h.a,{className:se.info},r.a.createElement("div",{className:"my-4"},r.a.createElement(f.a,{variant:"h6"},"Network Information")),r.a.createElement(g.a,{container:!0,direction:"row",justify:"center",alignItems:"center"},r.a.createElement(g.a,{item:!0,xs:12,sm:12,md:6},r.a.createElement(R.a,{className:"pb-4"},r.a.createElement(S.a,{"aria-label":"simple table"},r.a.createElement(O.a,null,r.a.createElement(C.a,null,r.a.createElement(j.a,{scope:"row"},"Network Name"),r.a.createElement(j.a,{align:"right"},"Rowma Public Network")),r.a.createElement(C.a,null,r.a.createElement(j.a,{scope:"row"},"Network Type"),r.a.createElement(j.a,{align:"right"},"Public")),r.a.createElement(C.a,null,r.a.createElement(j.a,{scope:"row"},"Network URL"),r.a.createElement(j.a,{align:"right"},"https://rocky-peak-54058.herokuapp.com")),r.a.createElement(C.a,null,r.a.createElement(j.a,{scope:"row"},"Network Location"),r.a.createElement(j.a,{align:"right"},"US")),r.a.createElement(C.a,null,r.a.createElement(j.a,{scope:"row"},"Network Owner"),r.a.createElement(j.a,{align:"right"},r.a.createElement("a",{href:"https://asmsuechan.com"},"asmsuechan")))))))))),r.a.createElement(g.a,{item:!0,xs:12},r.a.createElement(b.a,{className:se.footer,fontSize:16},r.a.createElement(v.a,{className:se.footerLink,href:"https://rowma.github.io/documentation/en/getting-started"},"Documentation"),r.a.createElement(v.a,{className:se.footerLink,href:"https://github.com/rowma/rowma"},"GitHub")))))))};Boolean("localhost"===window.location.hostname||"[::1]"===window.location.hostname||window.location.hostname.match(/^127(?:\.(?:25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)){3}$/));t(140);o.a.render(r.a.createElement(D,null),document.getElementById("root")),"serviceWorker"in navigator&&navigator.serviceWorker.ready.then((function(e){e.unregister()})).catch((function(e){console.error(e.message)}))},76:function(e,a,t){e.exports=t(141)},81:function(e,a,t){},82:function(e,a,t){}},[[76,1,2]]]);
//# sourceMappingURL=main.f03df472.chunk.js.map