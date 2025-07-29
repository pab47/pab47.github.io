fetch('robots.json')
	.then(response => response.json())
	.then(robots => {
		const tableBody = document.querySelector('#robots-table tbody');

		robots.forEach(robot => {
			const tr = document.createElement('tr');
			
			// Name
			const tdName = document.createElement('td');
			tdName.textContent = robot.name;
			tdName.style.fontWeight = 'bold';

			// Robot Image and Link
			const tdImage = document.createElement('td');

			const robotImage = document.createElement('img');
			robotImage.src = robot.img;
			robotImage.alt = `${robot.name}`;
			robotImage.style.maxWidth = '100%';
			robotImage.style.borderRadius = '8px';
			robotImage.style.display = 'block';
			tdImage.appendChild(robotImage);

			const a = document.createElement('a');
			a.href = robot.link;
			a.target = '_blank';
			a.appendChild(robotImage);
			tdImage.appendChild(a);

			// Append to table row
			tr.appendChild(tdName);
			tr.appendChild(tdImage);
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