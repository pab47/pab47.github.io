fetch('alumni-undergrads.json')
	.then(response => response.json())
	.then(projects => {
		const tableBody = document.querySelector('#student-table tbody');

		projects.forEach(project => {
			const tr = document.createElement('tr');

			// Title
			const tdTitle = document.createElement('td');
			tdTitle.textContent = project.title;
			tdTitle.style.fontWeight = 'bold';

			// Years
			const tdYears = document.createElement('td');
			tdYears.textContent = project.years;

			// Major
			const tdMajor = document.createElement('td');
			tdMajor.textContent = project.major;

			// Links
			const tdLinks = document.createElement('td');
			project.links.forEach(link => {
				const a = document.createElement('a');
				const br = document.createElement('br');
				a.href = link.url;
				a.textContent = link.label;
				a.target = '_blank';
				tdLinks.appendChild(a);
				tdLinks.appendChild(br);
			});

			// People
			const tdImages = document.createElement('td');

			const container = document.createElement('div');
			container.style.display = 'flex';
			container.style.flexWrap = 'wrap';
			container.style.gap = '16px';

			project.people.forEach(person => {
				const wrapper = document.createElement('div');
				wrapper.style.display = 'flex';
				wrapper.style.flexDirection = 'column';
				wrapper.style.alignItems = 'center';
				wrapper.style.width = '50px';
				wrapper.style.marginLeft = '5px';

				const img = document.createElement('img');
				img.src = person.img;
				img.alt = `${person.name}`;
				img.style.maxWidth = '100%';
				img.style.borderRadius = '8px';
				img.style.display = 'block';

				const name = document.createElement('div');
				name.textContent = person.name;
				name.style.fontSize = '14px';
				name.style.marginTop = '6px';
				name.style.textAlign = 'center';

				wrapper.appendChild(img);
				wrapper.appendChild(name);
				container.appendChild(wrapper);
			});
			tdImages.appendChild(container);

			// Append to table row
			tr.appendChild(tdTitle);
			tr.appendChild(tdYears);
			tr.appendChild(tdMajor);
			tr.appendChild(tdLinks);
			tr.appendChild(tdImages);
			tableBody.appendChild(tr);

		});

		//Search Bar Logic
		document.getElementById('searchBar').addEventListener('input', function () {
			const query = this.value.toLowerCase();
			const rows = document.querySelectorAll('table tbody tr');

			rows.forEach(row => {
				const text = row.textContent.toLowerCase();
				row.style.display = text.includes(query) ? '' : 'none';
			});
		});

	})
	.catch(error => {
		console.error('Failed to load data:', error);
	});