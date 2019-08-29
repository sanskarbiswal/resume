//Modal Content for Contact

function contact_Modal(){
	var modal=document.getElementById("contactModal");
	var btn=document.getElementById("contactModLanch");
	var span=document.getElementsByClassName("close")[0];
	
	btn.onclick=function(){
		modal.style.display="block";
	}
	
	span.onclick=function(){
		modal.style.display="none";
	}
	
	window.onclick=function(event){
		if(event.target==modal){
			modal.style.display="none";
		}
	}
}