fetch('research.json')
	.then(response => response.json())
	.then(research => {
		research.forEach(paper => {
			const section = document.getElementById(`${paper.year}`);
			const div = document.createElement('div');
			div.classList.add('publication');
			div.textContent = paper.title;
			div.textContent += ' ';
			paper.links.forEach(link => {
				const a = document.createElement('a');
				a.href = link.url;
				a.textContent = link.label;
				a.target = '_blank';
				div.appendChild(a);
				div.innerHTML += '&nbsp;';
			});
			section.appendChild(div);
		});

		// Search bar logic
    const searchBar = document.getElementById('searchBar');
		const publications = document.querySelectorAll('.publication');
    searchBar.addEventListener('input', () => {
      const query = searchBar.value.toLowerCase();
      publications.forEach(pub => {
				const text = pub.textContent.toLowerCase();
				pub.classList.toggle('hide', !text.includes(query));
      });
    });

	})
	.catch(error => {
		console.error('Failed to load data:', error);
	});