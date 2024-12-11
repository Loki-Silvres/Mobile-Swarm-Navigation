// Listen for the "Enter" key to send the message
document.getElementById('user-input').addEventListener('keypress', function(event) {
    if (event.key === 'Enter') {
        event.preventDefault(); // Prevent the default behavior (e.g., creating a new line)
        sendMessage();
    }
});

function sendMessage() {
    const userInput = document.getElementById('user-input').value;
    if (!userInput) return;  // Ignore if input is empty

    // Append the user's message
    appendMessage(userInput, 'user');
    document.getElementById('user-input').value = '';  // Clear the input field

    // Send the user message to the backend (Flask)
    fetch('/get_response', {
        method: 'POST',
        headers: {
            'Content-Type': 'application/json'
        },
        body: JSON.stringify({ message: userInput })
    })
    .then(response => response.json())
    .then(data => {
        const botMessage = data.response;
        // Display the bot's message gradually
        displayMessageGradually(botMessage, 'bot');
    })
    .catch(err => console.log('Error:', err));
}

function appendMessage(message, sender) {
    const chatWindow = document.getElementById('chat-window');
    const messageElement = document.createElement('div');
    messageElement.classList.add('message', `${sender}-message`);
    messageElement.innerHTML = formatMessage(message);
    
    chatWindow.appendChild(messageElement);
    chatWindow.scrollTop = chatWindow.scrollHeight;
}

function displayMessageGradually(message, sender) {
    const chatWindow = document.getElementById('chat-window');
    const messageElement = document.createElement('div');
    messageElement.classList.add('message', `${sender}-message`);
    messageElement.innerHTML = ''; // Start with an empty message element

    chatWindow.appendChild(messageElement);
    chatWindow.scrollTop = chatWindow.scrollHeight;

    let index = 0;
    const interval = setInterval(() => {
        messageElement.innerHTML += message.charAt(index);
        index++;

        if (index === message.length) {
            clearInterval(interval); // Stop after all characters are added
        }
    }, 20); // 50 ms delay between each character
}

function formatMessage(message) {
    return `<div class="tree-node">${message}</div>`;
}
