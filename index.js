const colors = {
	0: 'cyan', // Sunday - Cyan
	1: 'lightgreen', // Monday - Light Green
	2: 'yellow', // Tuesday - Yellow
	3: 'lightblue', // Wednesday - Light Blue
	4: 'orange', // Thursday - Orange
	5: 'pink', // Friday - Pink
	6: 'purple', // Saturday - Purple
}

const today = new Date().getDay();
const box = document.getElementById('daily-color-box');
box.style.backgroundColor = colors[today];