const searchBar = document.getElementById('searchBar');
// const publications = document.querySelectorAll('.publication');
const videos = document.querySelectorAll('.videoContainer');

searchBar.addEventListener('input', () => {
	const query = searchBar.value.toLowerCase();
	// publications.forEach(pub => {
	// 	const text = pub.textContent.toLowerCase();
	// 	pub.classList.toggle('hide', !text.includes(query));
	// });
	videos.forEach(video => {
		const text = video.textContent.toLowerCase();
		video.classList.toggle('hide', !text.includes(query));
	});
});