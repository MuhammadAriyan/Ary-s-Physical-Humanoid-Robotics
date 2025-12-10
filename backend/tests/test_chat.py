import pytest
from fastapi.testclient import TestClient
from unittest.mock import AsyncMock, patch
from backend.app.main import app


@pytest.fixture
def client():
    with TestClient(app) as test_client:
        yield test_client


def test_health_check(client):
    response = client.get("/health")
    assert response.status_code == 200
    assert response.json() == {"status": "healthy"}


def test_chat_endpoint_exists(client):
    # Test that the chat endpoint exists (without actually processing a message)
    # This would require mocking the agent in a full implementation
    response = client.post("/api/chat", json={"message": "Hello"})
    # We expect this to fail due to missing agent implementation in test environment,
    # but the route should exist
    assert response.status_code in [200, 422, 500]  # 422 for validation error, 500 for agent error


def test_chat_stream_endpoint_exists(client):
    response = client.post("/api/chat/stream", json={"message": "Hello"})
    # Check that the streaming endpoint exists
    assert response.status_code in [200, 422, 500]


@pytest.mark.asyncio
async def test_fubuni_agent_process_message():
    """Test the Fubuni agent's process_message method directly"""
    from backend.app.agents.fubuni_agent import fubuni_agent

    # Mock the agent's run method to avoid API calls
    with patch.object(fubuni_agent.runner, 'run', new=AsyncMock()) as mock_run:
        mock_run.return_value = type('MockResult', (), {'output': 'Test response'})()

        result = await fubuni_agent.process_message("Test message")
        assert result == "Test response"