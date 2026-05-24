async function loadData() {

    try {
        console.log("Loading JSON...");

        const response = await fetch('unitree.json?t=' + new Date().getTime());

        console.log("HTTP status:", response.status);

        const data = await response.json();

        console.log("Loaded data:", data);

        displayData(data);

        document.getElementById('search').addEventListener('input', e => {

            const term = e.target.value.toLowerCase();

            const filtered = data.filter(item =>
                (item.title || "").toLowerCase().includes(term) ||
                (item.notes || "").toLowerCase().includes(term)
            );

            displayData(filtered);
        });

    } catch (err) {
        console.error("Failed to load JSON:", err);
    }
}


/**
 * Convert URLs and local files into clickable links
 */
function makeLinksClickable(text) {

    if (!text) return "";

    // 1. Convert web URLs
    const urlPattern = /(https?:\/\/[^\s]+)/g;

    text = text.replace(urlPattern, url =>
        `<a href="${url}" target="_blank" rel="noopener noreferrer">${url}</a>`
    );

    // 2. Convert local files (.cpp, .png, .pdf, etc.)
    const filePattern = /([a-zA-Z0-9_\-\/]+\.(cpp|png|jpg|jpeg|pdf|txt|md))/g;

    text = text.replace(filePattern, file => {
        return `<a href="${file}" target="_blank">${file}</a>`;
    });

    return text;
}


/**
 * Render cards
 */
function displayData(data) {

    const container = document.getElementById('content');

    container.innerHTML = '';

    data.forEach(item => {

        const title = item.title || "";
        const notes = makeLinksClickable(item.notes || "");

        container.innerHTML += `
            <div class="card">
                <h3>${title}</h3>
                <p style="white-space: pre-line;">${notes}</p>
            </div>
        `;
    });
}


// Start app
loadData();