fetch('alumni-graduates.json')
	.then(response => response.json())
	.then(people => {
		const tableBody = document.querySelector('#student-table tbody');

		people.forEach(person => {
			const tr = document.createElement('tr');

			// Img
			const tdImage = document.createElement('td');
			const img = document.createElement('img');
			img.src = person.img;
			img.alt = `${person.name}`;
			img.style.maxWidth = '100%';
			img.style.width = '100px'
			img.style.borderRadius = '8px';
			img.style.display = 'block';
			tdImage.appendChild(img);

			// Name
			const tdName = document.createElement('td');
			tdName.textContent = person.name;
			tdName.style.fontWeight = 'bold';

			// Years
			const tdYears = document.createElement('td');
			tdYears.textContent = person.years;

			// Study
			const tdStudy = document.createElement('td');
			tdStudy.textContent = person.study;

			// Links
			const tdLinks = document.createElement('td');
			person.links.forEach(link => {
				const a = document.createElement('a');
				const br = document.createElement('br');
				a.href = link.url;
				a.textContent = link.label;
				a.target = '_blank';
				tdLinks.appendChild(a);
				tdLinks.appendChild(br);
			});

			// Append to table row
			tr.appendChild(tdImage);
			tr.appendChild(tdName);
			tr.appendChild(tdYears);
			tr.appendChild(tdStudy);
			tr.appendChild(tdLinks);
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